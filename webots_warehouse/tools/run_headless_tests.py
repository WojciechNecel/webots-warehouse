# -*- coding: utf-8 -*-
"""
================================================================================
run_headless_tests.py — harness testow headless (CI) dla magazynu Webots
================================================================================

PRZEZNACZENIE
-------------
Harness uruchamia pelna symulacje magazynu w trybie headless
(``webots --batch --no-rendering --mode=fast --stdout --stderr``) jako
podproces, sledzi pojawianie sie raportow ``reports/report_<order_id>.json``
generowanych przez kontroler po zakonczeniu kazdego zamowienia, a nastepnie
wykonuje twarde ASERCJE na zebranych raportach i wypisuje wynik PASS/FAIL
na stdout. Kod wyjscia: 0 = wszystkie asercje zaliczone, 1 = FAIL.

Narzedzie jest samodzielne (tylko stdlib) i dziala z dowolnego katalogu —
korzen projektu wyznaczany jest jako katalog nadrzedny ``tools/``.

SPOSOB UZYCIA
-------------
    python tools/run_headless_tests.py                      # pelny test (dlugi!)
    python tools/run_headless_tests.py --timeout 1200       # wlasny limit czasu
    python tools/run_headless_tests.py --world worlds/warehouse.wbt
    python tools/run_headless_tests.py --port 1301          # port TCP Webots
    python tools/run_headless_tests.py --webots "C:\\...\\webots.exe"
    python tools/run_headless_tests.py --self-test          # suchy przebieg
                                                            # logiki asercji
                                                            # (bez Webots, tempfile)

IZOLACJA RAPORTOW
-----------------
Kontroler zawsze zapisuje raporty do ``reports/`` wzgledem KORZENIA projektu
(sciezka absolutna liczona od pliku kontrolera), wiec nie da sie przekierowac
zapisu z zewnatrz. Harness realizuje izolacje przez schowanie:

  1. PRZED testem wszystkie istniejace pliki ``reports/*.json`` sa
     przenoszone do tymczasowego katalogu (stash) — katalog ``reports/``
     startuje pusty, wiec kazdy pojawiajacy sie raport pochodzi z testu.
  2. PO tescie raporty wygenerowane przez symulacje sa przenoszone do
     ``reports_ci/`` (artefakty przebiegu, katalog jest czyszczony przed
     kazdym startem), a schowane raporty wracaja na swoje miejsce.

Konsola Webots (stdout+stderr kontrolerow) jest zapisywana do
``reports_ci/webots_console.log``.

UWAGA O WSPOLBIEZNOSCI
----------------------
Webots moze byc jednoczesnie uruchomiony przez uzytkownika (GUI na
domyslnym porcie 1234). Harness domyslnie uzywa osobnego portu TCP
``--port 1301``, zeby instancja testowa nie kolidowala z interaktywna.

OCZEKIWANIA (KONFIGURACJA)
--------------------------
Struktura ``EXPECTED_ORDERS`` na gorze pliku: dla kazdego zamowienia
oczekiwana flaga (``green``/``red``) i minimalna liczba kursow (``trips``).
Brak raportu = FAIL z podaniem powodu (timeout / wyjscie procesu Webots).

Brak dowolnego raportu oraz awaryjne zatrzymanie z ladunkiem sa bledami testu.

STRUKTURY DANYCH
----------------
EXPECTED_ORDERS : list[dict]
    Klucze: order_id (str), flag ("green"|"red"), min_trips (int),
    note (str — opis scenariusza).
OrderResult (dataclass)
    Wynik asercji jednego zamowienia: order_id, ok, details (list[str]).
================================================================================
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

# Korzen projektu: webots_warehouse/ (katalog nadrzedny tools/)
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# Konfiguracja harnessu
# ---------------------------------------------------------------------------

DEFAULT_TIMEOUT_S = 900          # limit czasu sciennego pelnego przebiegu [s]
DEFAULT_PORT = 1301              # port TCP Webots (GUI uzywa domyslnego 1234)
DEFAULT_WORLD = "worlds/warehouse.wbt"
POLL_INTERVAL_S = 2.0            # odstep odpytywania katalogu raportow [s]
KILL_GRACE_S = 10.0              # czas na grzeczne zamkniecie przed kill

# Kandydaci na sciezke webots.exe (pierwszy istniejacy wygrywa);
# mozna nadpisac opcja --webots lub zmienna srodowiskowa WEBOTS_EXE.
WEBOTS_EXE_CANDIDATES = [
    r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    r"C:\Program Files (x86)\Webots\msys64\mingw64\bin\webots.exe",
]

# ---------------------------------------------------------------------------
# Oczekiwania testowe — zrodlo prawdy dla asercji
# ---------------------------------------------------------------------------
# ORD-001: 4 pozycje, 13.0 kg — jeden kurs, wszystko zgodne      -> green
# ORD-002: 7 pozycji, 85.0 kg — przekracza limit platformy 50 kg -> green,
#          wymusza split ladunku, wiec trips >= 2
# ORD-003: zawiera P005 (brak na polce) i P020 (na polce lezy P999) -> red

EXPECTED_ORDERS: list[dict] = [
    {
        "order_id": "ORD-001",
        "flag": "green",
        "min_trips": 1,
        "note": "scenariusz TS-01: pelna zgodnosc, pojedynczy kurs",
    },
    {
        "order_id": "ORD-002",
        "flag": "green",
        "min_trips": 2,
        "note": "scenariusz TS-02: 85 kg > limit 50 kg — split ladunku, trips>=2",
    },
    {
        "order_id": "ORD-003",
        "flag": "red",
        "min_trips": 1,
        "note": "scenariusz TS-03: brak P005 + podmiana P020->P999 — flaga red",
    },
]

KNOWN_DEFECT_MISSING_REPORT = {}

EPS = 1e-6  # tolerancja porownan zmiennoprzecinkowych (nieuzywana rezerwa)


@dataclass
class OrderResult:
    """Wynik asercji jednego zamowienia: id, status zbiorczy, komunikaty."""
    order_id: str
    ok: bool = True
    details: list[str] = field(default_factory=list)

    def fail(self, message: str) -> None:
        """Rejestruje blad i ustawia status na FAIL."""
        self.ok = False
        self.details.append(f"  [BLAD] {message}")

    def note(self, message: str) -> None:
        """Rejestruje komunikat informacyjny (nie wplywa na status)."""
        self.details.append(f"  [INFO] {message}")


# ---------------------------------------------------------------------------
# Logika asercji (czysta — testowalna na sztucznych raportach)
# ---------------------------------------------------------------------------

def assert_report(expectation: dict, report: dict | None,
                  missing_reason: str = "") -> OrderResult:
    """Sprawdza jeden raport wzgledem oczekiwan.

    expectation — wpis z EXPECTED_ORDERS (order_id, flag, min_trips, note).
    report      — sparsowany JSON raportu lub None, gdy raport nie powstal
                  (wtedy missing_reason opisuje przyczyne: timeout, wyjscie
                  procesu Webots itp.).
    """
    order_id = expectation["order_id"]
    res = OrderResult(order_id)
    res.note(f"oczekiwano: flag={expectation['flag']}, "
             f"trips>={expectation['min_trips']} ({expectation['note']})")

    if report is None:
        reason = missing_reason or "raport nie powstal"
        defect = KNOWN_DEFECT_MISSING_REPORT.get(order_id)
        if defect:
            reason = f"{reason} — {defect}"
        res.fail(f"Brak raportu: {reason}.")
        return res

    # Flaga zamowienia
    actual_flag = report.get("flag")
    if actual_flag != expectation["flag"]:
        res.fail(f"flaga '{actual_flag}' != oczekiwana '{expectation['flag']}'.")
    else:
        res.note(f"flaga '{actual_flag}' — zgodna.")

    # Liczba kursow (split ladunku)
    actual_trips = report.get("trips")
    if not isinstance(actual_trips, int):
        res.fail(f"pole 'trips' ma nieprawidlowa wartosc: {actual_trips!r}.")
    elif actual_trips < expectation["min_trips"]:
        res.fail(f"trips={actual_trips} < oczekiwane >= {expectation['min_trips']}.")
    else:
        res.note(f"trips={actual_trips} — zgodne (>= {expectation['min_trips']}).")

    # Spojnosc statusow pozycji z flaga: green <=> wszystkie ok,
    # red <=> przynajmniej jedna pozycja nie-ok. Kontrola krzyzowa raportu.
    items = report.get("items")
    if isinstance(items, list) and items:
        statuses = {item.get("status") for item in items}
        bad = statuses - {"ok"}
        if actual_flag == "green" and bad:
            res.fail(f"flaga green, ale pozycje maja statusy: {sorted(bad)}.")
        if actual_flag == "red" and not bad:
            res.fail("flaga red, ale wszystkie pozycje maja status 'ok'.")
        res.note(f"pozycje: {len(items)}, statusy: {sorted(statuses)}.")
    else:
        res.note("raport nie zawiera pozycji 'items' — pominito kontrole krzyzowa.")

    return res


def evaluate_reports(reports_dir: Path,
                     expectations: list[dict] = EXPECTED_ORDERS,
                     missing_reason: str = "timeout harnessu") -> list[OrderResult]:
    """Wczytuje raporty z katalogu i wykonuje asercje dla wszystkich oczekiwan.

    Brakujacy lub uszkodzony plik raportu jest traktowany jak brak raportu
    (FAIL z powodem), nigdy wyjatek.
    """
    results: list[OrderResult] = []
    for exp in expectations:
        order_id = exp["order_id"]
        path = reports_dir / f"report_{order_id}.json"
        report: dict | None = None
        reason = missing_reason
        if path.is_file():
            try:
                report = json.loads(path.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError, UnicodeDecodeError) as exc:
                reason = f"raport nieczytelny ({type(exc).__name__}: {exc})"
        results.append(assert_report(exp, report, reason))
    return results


# ---------------------------------------------------------------------------
# Izolacja katalogu raportow (schowek przed testem, przywrocenie po)
# ---------------------------------------------------------------------------

class ReportsIsolation:
    """Schowuje istniejace raporty na czas testu i przywraca je po nim.

    use()     — przenosi reports/*.json do katalogu tymczasowego,
    restore() — przenosi raporty wygenerowane przez test do reports_ci/
                (artefakty), a schowane raporty z powrotem do reports/.
    """

    def __init__(self, reports_dir: Path, artifacts_dir: Path) -> None:
        self._reports_dir = reports_dir
        self._artifacts_dir = artifacts_dir
        self._stash_dir: Path | None = None

    def use(self) -> None:
        self._reports_dir.mkdir(parents=True, exist_ok=True)
        stashed = sorted(self._reports_dir.glob("report_*.json"))
        if not stashed:
            return
        self._stash_dir = Path(tempfile.mkdtemp(prefix="reports_stash_"))
        for path in stashed:
            shutil.move(str(path), str(self._stash_dir / path.name))

    def restore(self) -> None:
        # Artefakty przebiegu: reports/ -> reports_ci/
        self._artifacts_dir.mkdir(parents=True, exist_ok=True)
        for path in sorted(self._reports_dir.glob("report_*.json")):
            shutil.move(str(path), str(self._artifacts_dir / path.name))
        # Przywrocenie schowanych raportow
        if self._stash_dir is not None:
            for path in sorted(self._stash_dir.glob("*.json")):
                shutil.move(str(path), str(self._reports_dir / path.name))
            shutil.rmtree(self._stash_dir, ignore_errors=True)
            self._stash_dir = None


# ---------------------------------------------------------------------------
# Uruchamianie i nadzorowanie procesu Webots
# ---------------------------------------------------------------------------

def find_webots_exe(cli_value: str | None) -> Path:
    """Wyznacza sciezke webots.exe: opcja CLI > env WEBOTS_EXE > kandydaci."""
    import os
    candidates = []
    if cli_value:
        candidates.append(cli_value)
    env_value = os.environ.get("WEBOTS_EXE")
    if env_value:
        candidates.append(env_value)
    candidates.extend(WEBOTS_EXE_CANDIDATES)
    for raw in candidates:
        path = Path(raw)
        if path.is_file():
            return path
    searched = "\n  ".join(candidates)
    raise FileNotFoundError(
        f"Nie znaleziono webots.exe. Przeszukano:\n  {searched}\n"
        "Podaj sciezke opcja --webots lub zmienna srodowiskowa WEBOTS_EXE."
    )


def launch_webots(webots_exe: Path, world_path: Path, port: int,
                  console_log: Path) -> subprocess.Popen:
    """Startuje Webots headless jako podproces; konsole kieruje do pliku.

    Plik (zamiast PIPE) zapobiega zakleszczeniu na pelnym buforze potoku,
    gdy kontrolery duzo wypisuja.
    """
    cmd = [
        str(webots_exe),
        "--batch",
        "--no-rendering",
        "--mode=fast",
        "--stdout",
        "--stderr",
        f"--port={port}",
        str(world_path),
    ]
    log_file = console_log.open("w", encoding="utf-8", errors="replace")
    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        cwd=str(PROJECT_ROOT),
    )
    proc._console_log_file = log_file  # domkniecie przy kill_webots
    return proc


def kill_webots(proc: subprocess.Popen) -> None:
    """Zamyka proces Webots: najpierw terminate, po grace okresie kill."""
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=KILL_GRACE_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=KILL_GRACE_S)
    log_file = getattr(proc, "_console_log_file", None)
    if log_file is not None:
        log_file.close()


def wait_for_reports(proc: subprocess.Popen, reports_dir: Path,
                     expectations: list[dict], timeout_s: float) -> str:
    """Czeka na raporty wszystkich oczekiwanych zamowien.

    Zwraca powod zakonczenia oczekiwania:
      "all_collected" — zebrano wszystkie raporty,
      "webots_exited" — proces Webots zakonczyl sie przed czasem,
      "timeout"       — przekroczono limit czasu sciennego.
    """
    expected_names = {f"report_{e['order_id']}.json" for e in expectations}
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        present = {p.name for p in reports_dir.glob("report_*.json")}
        if expected_names <= present:
            return "all_collected"
        if proc.poll() is not None:
            return "webots_exited"
        time.sleep(POLL_INTERVAL_S)
    return "timeout"


# ---------------------------------------------------------------------------
# Raport wynikow
# ---------------------------------------------------------------------------

def print_results(results: list[OrderResult], wait_reason: str,
                  elapsed_s: float) -> int:
    """Wypisuje raport PASS/FAIL na stdout i zwraca kod wyjscia 0/1."""
    print("\n" + "=" * 72)
    print("WYNIKI TESTOW HEADLESS — symulacja magazynu Webots")
    print(f"Zakonczenie oczekiwania: {wait_reason}; czas: {elapsed_s:.0f} s")
    print("=" * 72)
    passed = 0
    for res in results:
        status = "PASS" if res.ok else "FAIL"
        if res.ok:
            passed += 1
        print(f"\n[{status}] {res.order_id}")
        for line in res.details:
            print(line)
    print("\n" + "=" * 72)
    total = len(results)
    if passed == total:
        print(f"WYNIK: PASS — {passed}/{total} zamowien zgodnych z oczekiwaniami.")
        print("=" * 72)
        return 0
    print(f"WYNIK: FAIL — {passed}/{total} zaliczonych, {total - passed} nie.")
    print("=" * 72)
    return 1


# ---------------------------------------------------------------------------
# Suchy przebieg logiki asercji (--self-test, bez Webots, tempfile)
# ---------------------------------------------------------------------------

def _self_test() -> int:
    """Cwiczy evaluate_reports na sztucznych raportach w katalogu tymczasowym.

    Scenariusze:
      A — wszystkie raporty poprawne               -> wszystkie PASS,
    B — brak raportu ORD-003 (symulacja timeout) -> FAIL,
      C — ORD-001 z flaga red (niezgodnosc)        -> FAIL flagi,
      D — ORD-002 z trips=1 (brak splitu)          -> FAIL trips.
    Zwraca 0, gdy logika asercji zachowala sie zgodnie z oczekiwaniami.
    """
    def make_report(order_id: str, flag: str, trips: int, bad: bool = False) -> dict:
        items = [{"product_id": "P001", "status": "ok", "weight": 3.5,
                  "message": "pobrano pomyslnie"}]
        if bad:
            items.append({"product_id": "P005", "status": "missing",
                          "weight": None, "message": "brak produktu na polce"})
        return {"order_id": order_id, "flag": flag, "trips": trips,
                "timestamp": "2026-01-01T00:00:00", "items": items, "logs": []}

    failures: list[str] = []
    with tempfile.TemporaryDirectory() as tmp:
        reports_dir = Path(tmp)

        # Scenariusz A: pelny zestaw poprawnych raportow
        for oid, flag, trips, bad in (
            ("ORD-001", "green", 1, False),
            ("ORD-002", "green", 2, False),
            ("ORD-003", "red", 1, True),
        ):
            (reports_dir / f"report_{oid}.json").write_text(
                json.dumps(make_report(oid, flag, trips, bad)), encoding="utf-8")
        results = evaluate_reports(reports_dir)
        if not all(r.ok for r in results):
            failures.append("A: poprawne raporty nie daly samych PASS: "
                            + "; ".join(f"{r.order_id}={r.ok}" for r in results))

        # Scenariusz B: brak ORD-003 -> FAIL
        (reports_dir / "report_ORD-003.json").unlink()
        results = evaluate_reports(reports_dir)
        ord3 = next(r for r in results if r.order_id == "ORD-003")
        if ord3.ok:
            failures.append("B: brak raportu ORD-003 nie dal FAIL.")
        if not all(r.ok for r in results if r.order_id != "ORD-003"):
            failures.append("B: pozostale raporty powinny dawac PASS.")

        # Scenariusz C: zla flaga ORD-001
        (reports_dir / "report_ORD-003.json").write_text(
            json.dumps(make_report("ORD-003", "red", 1, bad=True)), encoding="utf-8")
        (reports_dir / "report_ORD-001.json").write_text(
            json.dumps(make_report("ORD-001", "red", 1, bad=True)), encoding="utf-8")
        results = evaluate_reports(reports_dir)
        ord1 = next(r for r in results if r.order_id == "ORD-001")
        if ord1.ok or not any("flaga" in d for d in ord1.details):
            failures.append("C: zla flaga ORD-001 nie wykryta.")

        # Scenariusz D: ORD-002 bez splitu (trips=1)
        (reports_dir / "report_ORD-001.json").write_text(
            json.dumps(make_report("ORD-001", "green", 1)), encoding="utf-8")
        (reports_dir / "report_ORD-002.json").write_text(
            json.dumps(make_report("ORD-002", "green", 1)), encoding="utf-8")
        results = evaluate_reports(reports_dir)
        ord2 = next(r for r in results if r.order_id == "ORD-002")
        if ord2.ok or not any("trips" in d for d in ord2.details):
            failures.append("D: trips=1 dla ORD-002 nie wykryte.")

        # Scenariusz E: uszkodzony plik JSON -> FAIL, nie wyjatek
        (reports_dir / "report_ORD-002.json").write_text(
            "{nie-json", encoding="utf-8")
        results = evaluate_reports(reports_dir)
        ord2 = next(r for r in results if r.order_id == "ORD-002")
        if ord2.ok or not any("nieczytelny" in d for d in ord2.details):
            failures.append("E: uszkodzony raport nie dal FAIL z powodem.")

        # Pokazowy wydruk asercji na sztucznych danych (scenariusz A odtworzony)
        for oid, flag, trips, bad in (
            ("ORD-001", "green", 1, False),
            ("ORD-002", "green", 2, False),
            ("ORD-003", "red", 1, True),
        ):
            (reports_dir / f"report_{oid}.json").write_text(
                json.dumps(make_report(oid, flag, trips, bad)), encoding="utf-8")
        demo = evaluate_reports(reports_dir)
        print("--- Pokazowy przebieg asercji (sztuczne raporty, tempfile) ---")
        for res in demo:
            print(f"[{'PASS' if res.ok else 'FAIL'}] {res.order_id}")
            for line in res.details:
                print(line)

    if failures:
        print("\nSELF-TEST: FAIL")
        for f in failures:
            print(f"  [BLAD] {f}")
        return 1
    print("\nSELF-TEST: PASS — logika asercji dziala zgodnie z oczekiwaniami "
          "(scenariusze A-E).")
    return 0


# ---------------------------------------------------------------------------
# Glowny przebieg harnessu
# ---------------------------------------------------------------------------

def run_harness(args: argparse.Namespace) -> int:
    """Pelny przebieg: izolacja -> Webots headless -> asercje -> raport."""
    world_path = Path(args.world)
    if not world_path.is_absolute():
        world_path = PROJECT_ROOT / world_path
    if not world_path.is_file():
        print(f"[BLAD] Brak pliku swiata: {world_path}", file=sys.stderr)
        return 1

    try:
        webots_exe = find_webots_exe(args.webots)
    except FileNotFoundError as exc:
        print(f"[BLAD] {exc}", file=sys.stderr)
        return 1

    reports_dir = PROJECT_ROOT / "reports"
    artifacts_dir = PROJECT_ROOT / "reports_ci"
    # Czysty katalog artefaktow na czas testu
    shutil.rmtree(artifacts_dir, ignore_errors=True)
    artifacts_dir.mkdir(parents=True, exist_ok=True)
    console_log = artifacts_dir / "webots_console.log"
    fsm_log = PROJECT_ROOT / "logs" / "fsm_debug.log"
    fsm_start = fsm_log.stat().st_size if fsm_log.exists() else 0

    isolation = ReportsIsolation(reports_dir, artifacts_dir)
    isolation.use()

    print("=" * 72)
    print("HARNESS TESTOW HEADLESS (CI)")
    print(f"  Webots : {webots_exe}")
    print(f"  Swiat  : {world_path}")
    print(f"  Port   : {args.port} (osobny od GUI=1234)")
    print(f"  Timeout: {args.timeout} s")
    print(f"  Konsola: {console_log}")
    print("=" * 72)
    print("Startuje Webots headless i czekam na raporty: "
          + ", ".join(e["order_id"] for e in EXPECTED_ORDERS))

    proc = launch_webots(webots_exe, world_path, args.port, console_log)
    start = time.monotonic()
    try:
        wait_reason = wait_for_reports(proc, reports_dir, EXPECTED_ORDERS,
                                       args.timeout)
    finally:
        kill_webots(proc)
        isolation.restore()
    elapsed = time.monotonic() - start

    missing_reason = {
        "timeout": f"przekroczono limit {args.timeout} s czasu sciennego",
        "webots_exited": "proces Webots zakonczyl sie przed wygenerowaniem raportu",
        "all_collected": "raport nie powstal mimo zebrania pozostalych",
    }[wait_reason]
    results = evaluate_reports(artifacts_dir, EXPECTED_ORDERS, missing_reason)
    if fsm_log.exists():
        with fsm_log.open("rb") as stream:
            stream.seek(fsm_start)
            new_log = stream.read().decode("utf-8", errors="replace")
        if "[FSM] AWARIA:" in new_log:
            failure = OrderResult("ROZLADUNEK")
            failure.fail("Kontroler zakonczyl kurs z ladunkiem na robocie.")
            results.append(failure)
    return print_results(results, wait_reason, elapsed)


def main() -> int:
    """Parsuje argumenty CLI i uruchamia self-test albo pelny harness."""
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(
        description="Harness testow headless (CI) symulacji magazynu Webots. "
                    "Uruchamia Webots bez renderingu, zbiera raporty z reports/ "
                    "i wykonuje asercje per zamowienie (exit 0=PASS, 1=FAIL).",
    )
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help=f"limit czasu sciennego przebiegu [s] "
                             f"(domyslnie {DEFAULT_TIMEOUT_S})")
    parser.add_argument("--world", default=DEFAULT_WORLD,
                        help=f"sciezka do pliku swiata (domyslnie {DEFAULT_WORLD})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"port TCP Webots (domyslnie {DEFAULT_PORT}; "
                             f"GUI uzywa 1234)")
    parser.add_argument("--webots", default=None,
                        help="sciezka do webots.exe (domyslnie autodetekcja)")
    parser.add_argument("--self-test", action="store_true",
                        help="suchy przebieg logiki asercji na sztucznych "
                             "raportach (bez Webots, katalog tymczasowy)")
    args = parser.parse_args()

    if args.self_test:
        return _self_test()
    return run_harness(args)


if __name__ == "__main__":
    sys.exit(main())
