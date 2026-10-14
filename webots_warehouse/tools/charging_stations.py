"""Charging bays shared by the world generator and existing-world installer."""


def charging_station_nodes(layout):
    cells = layout.get('charging_cells', [[15, 11], [0, 11]])
    origin = layout.get('origin', {'x': -7.5, 'z': -5.5})
    nodes = []
    for index, (c, r) in enumerate(cells, 1):
        x, z = origin['x'] + c, origin['z'] + r
        nodes.append(f'''DEF CHARGER_{index} Solid {{
  translation {x} 0 {z}
  name "charging_station_{index}"
  children [
    Pose {{
      translation 0 0.004 0
      children [ Shape {{
        appearance PBRAppearance {{ baseColor 0.03 0.32 0.38 roughness 0.8 }}
        geometry Box {{ size 0.88 0.008 0.90 }}
      }} ]
    }}
    Pose {{
      translation 0 0.50 1.05
      children [ Shape {{
        appearance PBRAppearance {{ baseColor 0.08 0.12 0.18 metalness 0.45 roughness 0.4 }}
        geometry Box {{ size 0.65 1.0 0.30 }}
      }} ]
    }}
    Pose {{
      translation 0 0.76 0.89
      children [ Shape {{
        appearance DEF CHARGER_LED_{index} PBRAppearance {{
          baseColor 0.1 0.9 0.4 emissiveColor 0.1 0.9 0.4
          emissiveIntensity 0.7 roughness 0.4
        }}
        geometry Box {{ size 0.40 0.20 0.02 }}
      }} ]
    }}
    Pose {{
      translation 0.22 0.76 0.89
      children [ Shape {{
        appearance PBRAppearance {{ baseColor 0.7 0.9 0.9 }}
        geometry Box {{ size 0.04 0.08 0.02 }}
      }} ]
    }}
  ]
  boundingObject Pose {{
    translation 0 0.50 1.05
    children [ Box {{ size 0.65 1.0 0.30 }} ]
  }}
  locked TRUE
}}''')
    return '\n'.join(nodes) + '\n'
