# ============================================================================
# generate_world.py -- generator swiata symulacji magazynu (warehouse.wbt)
# ----------------------------------------------------------------------------
# PRZEZNACZENIE:
#   Generuje plik swiata Webots `worlds/warehouse.wbt` dla symulacyjnego
#   systemu obslugi zamowien magazynowych. Generator powstal jako zrodlo
#   prawdy o strukturze swiata -- pozwala odtworzyc swiat po przypadkowym
#   nadpisaniu go przez Webots (np. po zapisie stanu symulacji z blednymi
#   wezlami) i gwarantuje zgodnosc z danymi w `data/`.
#
# SPOSOB UZYCIA:
#   python tools/generate_world.py
#   Skrypt nadpisuje worlds/warehouse.wbt (wczesniej tworzy kopie
#   worlds/warehouse.wbt.bak, jesli plik istnieje). Dane wejsciowe:
#   - data/warehouse_layout.json -- siatka, komorki regalow, terminal,
#   - data/products.json         -- produkty (pozycje, wagi, tekstury QR).
#   Oba pliki generuje tools/generate_assets.py.
#
# STRUKTURA GENEROWANEGO SWIATA (Webots R2025a):
#   - EXTERNPROTO: TexturedBackground, TexturedBackgroundLight
#     (oficjalne PROTO Cyberbotics, wymagane deklaracje od R2023b+)
#     oraz lokalne ../protos/WarehouseRobot.proto.
#     UWAGA: deklaracje EXTERNPROTO musza byc na poczatku pliku, przed
#     wszystkimi wezlami -- inaczej Webots zglasza blad parsowania.
#   - Oswietlenie bezcieniowe: DirectionalLight + 12 x PointLight
#     (attenuation 0 0 1 -- kwadratowe tlumienie, zalecane przez Webots).
#   - Podloga 18 x 14 m + 4 sciany 2.5 m jako jawne statyczne Solidy
#     (RectangleArena z EXTERNPROTO nie dawala czynnych kolizji).
#   - 60 regalow (DEF SHELF_<c>_<r>, statyczne: 4 slupki + 3 polki),
#   - produkty (DEF PRODUCT_<id>, fizyka, masa = waga, tekstura QR;
#     P005 celowo pomijany -- scenariusz "brak produktu"; box na miejscu
#     P020 nosi teksture P999.png -- scenariusz "zly produkt na polce"),
#   - strefa terminala (DEF TERMINAL_ZONE) + kiosk (DEF TERMINAL),
#   - 3 przeszkody dynamiczne (DEF OBSTACLE_1..3),
#   - instancja robota WarehouseRobot (kontroler "warehouse_controller"),
#   - Robot "mcp_bridge" (supervisor TRUE) -- pomost dla serwera MCP
#     (https://github.com/MHasaan/webotMCP); wymaga skopiowania
#     controllers/mcp_bridge (patrz README.md, sekcja MCP).
#   Wszystkie wezly Solid maja unikalne pola `name` (Webots ostrzega
#   o duplikatach domyslnej nazwy "solid").
#
# UKLAD WSPOLRZEDNYCH: WorldInfo deklaruje jawnie coordinateSystem "NUE"
# (podloga = plaszczyzna XZ, os Y w gore) -- bez tej deklaracji R2025a
# przyjalby domyslne ENU (Z w gore) i cala geometria magazynu lezalaby
# "na boku". Uwaga: WorldInfo.gravity jest od R2023a SKALAREM (SFFloat),
# nie wektorem. Komorka (c,r): x = -7.5 + c, z = -5.5 + r.
# ============================================================================

import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "data" / "warehouse_layout.json"
PRODUCTS_PATH = ROOT / "data" / "products.json"
WORLD_PATH = ROOT / "worlds" / "warehouse.wbt"

# Wersja naglowka i adresy EXTERNPROTO zgodne z Webots R2025a.
WEBOTS_VERSION = "R2025a"
EXTERNPROTOS = [
    "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/"
    "projects/objects/backgrounds/protos/TexturedBackground.proto",
    "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/"
    "projects/objects/backgrounds/protos/TexturedBackgroundLight.proto",
    "../protos/WarehouseRobot.proto",
]

# Punkty oswietleniowe nad alejami (x, z); y = 2.7 m.
POINT_LIGHTS = [(x, z) for z in (-4.5, -1.5, 1.5, 4.5) for x in (-5.0, 0.0, 5.0)]

# Przeszkody dynamiczne (DEF OBSTACLE_<n>): pozycje (x, z) na alejach.
OBSTACLES = [(0.5, -1.5), (-4.5, 1.5), (2.5, 4.5)]

HEADER_COMMENT = """\
# ============================================================================
# warehouse.wbt -- swiat symulacji magazynu (Webots R2025a)
# ----------------------------------------------------------------------------
# Wygenerowany przez tools/generate_world.py -- NIE EDYTUJ RECZNIE,
# poprawki wprowadzaj w generatorze i regeneruj (python tools/generate_world.py).
# Plik definiuje: hale 18x14 m (podloga + 4 sciany, statyczne Solidy),
# 60 regalow pietrowych, produkty z kodami QR, strefe Terminala Zamowien,
# 3 przeszkody dynamiczne, robota WarehouseRobot (EXTERNPROTO) oraz
# robota-pomost mcp_bridge (MCP).
# Siatka nawigacyjna 16x12 komorek (1.0 m): srodek komorki (c,r):
# x = -7.5 + c, z = -5.5 + r. Szczegoly: docs/architecture.md.
# ============================================================================
"""


def pbr(base_color=None, roughness=0.8, metalness=0.0, texture_url=None,
        emissive=None, emissive_intensity=None, indent=6):
    """Buduje blok PBRAppearance (wciecie `indent` spacji)."""
    pad = " " * indent
    lines = [f"{pad}PBRAppearance {{"]
    if base_color is not None:
        lines.append(f"{pad}  baseColor {' '.join(str(v) for v in base_color)}")
    if texture_url is not None:
        lines.append(f'{pad}  baseColorMap ImageTexture {{')
        lines.append(f'{pad}    url [ "{texture_url}" ]')
        lines.append(f"{pad}  }}")
    lines.append(f"{pad}  roughness {roughness}")
    lines.append(f"{pad}  metalness {metalness}")
    if emissive is not None:
        lines.append(f"{pad}  emissiveColor {' '.join(str(v) for v in emissive)}")
        lines.append(f"{pad}  emissiveIntensity {emissive_intensity}")
    lines.append(f"{pad}}}")
    return "\n".join(lines)


def box_shape(size, appearance_block, indent):
    """Buduje Shape z geometria Box i podanym blokiem Appearance."""
    pad = " " * indent
    return (
        f"{pad}Shape {{\n"
        f"{pad}  appearance {appearance_block}\n"
        f"{pad}  geometry Box {{\n"
        f"{pad}    size {' '.join(str(v) for v in size)}\n"
        f"{pad}  }}\n"
        f"{pad}}}"
    )


def shelf_node(cell, name):
    """Regal statyczny: 4 slupki narozne + 3 polki (DEF SHELF_<c>_<r>).

    boundingObject obejmuje tylko slupki i polki (nie cala bryle), dzieki
    czemu produkty stojace na polkach nie koliduja z regalem.
    """
    c, r = cell
    x, z = -7.5 + c, -5.5 + r
    post_color = pbr((0.25, 0.28, 0.32), roughness=0.6, indent=10)
    board_color = pbr((0.55, 0.45, 0.35), roughness=0.8, indent=10)
    parts, bounds = [], []
    for px in (-0.425, 0.425):
        for pz in (-0.425, 0.425):
            parts.append(
                f"          Pose {{\n            translation {px} 0.9 {pz}\n"
                f"            children [\n{box_shape((0.05, 1.8, 0.05), post_color, 12)}\n"
                f"            ]\n          }}")
            bounds.append(
                f"      Pose {{\n        translation {px} 0.9 {pz}\n"
                f"        children [\n          Box {{ size 0.05 1.8 0.05 }}\n"
                f"        ]\n      }}")
    for level_y in (0.2, 0.8, 1.4):
        board_y = round(level_y - 0.145, 3)  # gorna powierzchnia = level - 0.125
        parts.append(
            f"          Pose {{\n            translation 0 {board_y} 0\n"
            f"            children [\n{box_shape((0.9, 0.04, 0.9), board_color, 12)}\n"
            f"            ]\n          }}")
        bounds.append(
            f"      Pose {{\n        translation 0 {board_y} 0\n"
            f"        children [\n          Box {{ size 0.9 0.04 0.9 }}\n"
            f"        ]\n      }}")
    return (
        f"DEF {name} Solid {{\n"
        f"  translation {x} 0 {z}\n"
        f'  name "{name}"\n'
        f"  children [\n"
        f"    Group {{\n      children [\n" + "\n".join(parts) + "\n      ]\n    }\n"
        f"  ]\n"
        f"  boundingObject Group {{\n    children [\n" + "\n".join(bounds) + "\n    ]\n  }\n"
        f"  locked TRUE\n"
        f"}}"
    )


def product_node(product):
    """Produkt: Solid 0.25 m z fizyka (mass = waga) i tekstura QR.

    Gdy produkt ma ustawione `world_qr` (anomalia "zly produkt na polce"),
    na bryle ląduje tekstura kodu z payloadu world_qr zamiast wlasnego.
    """
    pid = product["id"]
    x, y, z = product["position"]
    qr_id = product["world_qr"].split(";")[0] if product.get("world_qr") else pid
    appearance = pbr(roughness=0.7, texture_url=f"../textures/qr/{qr_id}.png",
                     indent=6)
    return (
        f"DEF PRODUCT_{pid} Solid {{\n"
        f"  translation {x} {y} {z}\n"
        f'  name "PRODUCT_{pid}"\n'
        f"  children [\n{box_shape((0.25, 0.25, 0.25), appearance, 4)}\n  ]\n"
        f"  boundingObject Box {{\n    size 0.25 0.25 0.25\n  }}\n"
        f"  physics Physics {{\n    density -1\n    mass {product['weight']}\n  }}\n"
        f"}}"
    )


def obstacle_node(index, x, z):
    """Przeszkoda dynamiczna: kostka 0.4 m z fizyka (DEF OBSTACLE_<n>)."""
    name = f"OBSTACLE_{index}"
    appearance = pbr((0.9, 0.35, 0.1), roughness=0.6, indent=6)
    return (
        f"DEF {name} Solid {{\n"
        f"  translation {x} 0.2 {z}\n"
        f'  name "{name}"\n'
        f"  children [\n{box_shape((0.4, 0.4, 0.4), appearance, 4)}\n  ]\n"
        f"  boundingObject Box {{\n    size 0.4 0.4 0.4\n  }}\n"
        f"  physics Physics {{\n    density -1\n    mass 5\n  }}\n"
        f"}}"
    )


def terminal_zone_node():
    """Zielony patch 2x2 m oznaczajacy strefe wydania (DEF TERMINAL_ZONE)."""
    appearance = pbr((0.05, 0.65, 0.2), roughness=1.0, indent=8)
    return (
        "DEF TERMINAL_ZONE Pose {\n"
        "  translation 0 0.01 5.5\n"
        "  children [\n"
        f"{box_shape((2, 0.02, 2), appearance, 6)}\n"
        "  ]\n}"
    )


def terminal_kiosk_node():
    """Kiosk Terminala Zamowien (DEF TERMINAL) -- statyczny, przy poludniowej scianie."""
    body = pbr((0.16, 0.2, 0.26), roughness=0.5, indent=8)
    screen = pbr((0.1, 0.35, 0.15), roughness=0.4, indent=8,
                 emissive=(0.1, 0.8, 0.3), emissive_intensity=0.6)
    return (
        "DEF TERMINAL Solid {\n"
        "  translation 0 0 6.6\n"
        '  name "TERMINAL"\n'
        "  children [\n"
        "    Pose {\n      translation 0 0.6 0\n      children [\n"
        f"{box_shape((0.8, 1.2, 0.35), body, 8)}\n      ]\n    }}\n"
        "    Pose {\n      translation 0 1.25 -0.1\n      rotation 1 0 0 -0.25\n"
        "      children [\n"
        f"{box_shape((0.6, 0.4, 0.05), screen, 8)}\n      ]\n    }}\n"
        "  ]\n"
        "  boundingObject Pose {\n    translation 0 0.6 0\n"
        "    children [\n      Box {\n        size 0.8 1.2 0.35\n      }\n    ]\n  }\n"
        "  locked TRUE\n}"
    )


def build_world(layout, products):
    """Sklada pelna tresc pliku warehouse.wbt (lista sekcji -> tekst)."""
    sections = [f"#VRML_SIM {WEBOTS_VERSION} utf8\n\n{HEADER_COMMENT}"]
    sections.extend(f'EXTERNPROTO "{url}"' for url in EXTERNPROTOS)
    sections.append("WorldInfo {\n  basicTimeStep 32\n"
                    '  coordinateSystem "NUE"\n'
        "  contactProperties [\n"
        "    ContactProperties {\n"
        '      material1 "tire"\n      material2 "floor"\n'
        "      coulombFriction 2.5\n      softCFM 0.001\n    }\n"
        "    ContactProperties {\n"
        '      material1 "body"\n      material2 "floor"\n'
        "      coulombFriction 0.05\n      softCFM 0.001\n    }\n"
        "  ]\n}")
    sections.append("Viewpoint {\n"
                    "  orientation 0.34216 0.34216 -0.87513 1.70379\n"
                    "  position 0 13 12\n}")
    sections.append("TexturedBackground {\n  skyColor [\n    0.45 0.58 0.72\n  ]\n}")
    sections.append("TexturedBackgroundLight {\n}")
    sections.append("DirectionalLight {\n  ambientIntensity 0.55\n"
                    "  color 1 1 0.98\n  direction -0.25 -1 -0.15\n"
                    "  intensity 0.85\n  castShadows FALSE\n}")
    for x, z in POINT_LIGHTS:
        sections.append(
            f"PointLight {{\n  attenuation 0 0 1\n"
            f"  intensity 0.55\n  location {x} 2.7 {z}\n  radius 7\n"
            f"  castShadows FALSE\n}}")
    # Podloga i sciany jako jawne statyczne Solidy (niezawodne kolizje --
    # RectangleArena z EXTERNPROTO nie dawala czynnego boundingObject
    # i dynamiczne obiekty przelatywaly przez podloge).
    floor_app = pbr((0.62, 0.62, 0.65), roughness=0.9, indent=6)
    wall_app = pbr((0.85, 0.86, 0.9), roughness=0.9, indent=6)
    sections.append(
        "DEF FLOOR Solid {\n"
        "  translation 0 -0.01 0\n"
        '  name "floor"\n'
        '  contactMaterial "floor"\n'
        "  children [\n" + box_shape((18, 0.02, 14), floor_app, 4) + "\n  ]\n"
        "  boundingObject Box {\n    size 18 0.02 14\n  }\n"
        "  locked TRUE\n}")
    walls = [
        ("WALL_N", (0, 1.25, -7.05), (18.2, 2.5, 0.1)),
        ("WALL_S", (0, 1.25, 7.05), (18.2, 2.5, 0.1)),
        ("WALL_W", (-9.05, 1.25, 0), (0.1, 2.5, 14.2)),
        ("WALL_E", (9.05, 1.25, 0), (0.1, 2.5, 14.2)),
    ]
    for wname, (wx, wy, wz), wsize in walls:
        sections.append(
            f"DEF {wname} Solid {{\n"
            f"  translation {wx} {wy} {wz}\n"
            f'  name "{wname.lower()}"\n'
            f"  children [\n{box_shape(wsize, wall_app, 4)}\n  ]\n"
            f"  boundingObject Box {{\n"
            f"    size {' '.join(str(v) for v in wsize)}\n  }}\n"
            f"  locked TRUE\n}}")
    sections.append(terminal_zone_node())
    sections.append(terminal_kiosk_node())
    for cell in layout["blocked_cells"]:
        sections.append(shelf_node(cell, f"SHELF_{cell[0]}_{cell[1]}"))
    for product in products:
        if product.get("present", True):
            sections.append(product_node(product))
    for i, (x, z) in enumerate(OBSTACLES, start=1):
        sections.append(obstacle_node(i, x, z))
    sections.append(
        "WarehouseRobot {\n  translation -0.5 0.01 5.5\n  rotation 0 1 0 0\n"
        '  controller "warehouse_controller"\n}')
    sections.append(
        "DEF MCP_BRIDGE Robot {\n  translation 0 0 -10\n"
        '  name "mcp_bridge"\n  controller "mcp_bridge"\n  supervisor TRUE\n}')
    return "\n".join(sections) + "\n"


def main():
    """Generuje worlds/warehouse.wbt z data/*.json (kopia .wbt.bak)."""
    layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    products = json.loads(PRODUCTS_PATH.read_text(encoding="utf-8"))["products"]
    if WORLD_PATH.exists():
        backup = WORLD_PATH.with_suffix(".wbt.bak")
        shutil.copy2(WORLD_PATH, backup)
        print(f"[BACKUP] {backup.relative_to(ROOT)}")
    text = build_world(layout, products)
    WORLD_PATH.write_text(text, encoding="utf-8")
    n_shelves = text.count("DEF SHELF_")
    n_products = text.count("DEF PRODUCT_")
    print(f"[OK] Zapisano {WORLD_PATH.relative_to(ROOT)} "
          f"({n_shelves} regalow, {n_products} produktow, "
          f"{len(OBSTACLES)} przeszkody, {len(POINT_LIGHTS)} PointLight)")


if __name__ == "__main__":
    main()
