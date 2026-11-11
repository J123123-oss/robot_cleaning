from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "rtk_nav"))

from geofence_geometry import build_geofence_payload


def test_payload_uses_inner_boundaries_and_bridge_tracks():
    payload = build_geofence_payload(
        ["E1", "E2"],
        [[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)],
         [(20.0, 0.0), (30.0, 0.0), (30.0, 5.0), (20.0, 5.0)]],
        [{
            "name": "E1_to_E2",
            "from_area": "E1",
            "to_area": "E2",
            "centerline_utm": [(10.0, 2.5), (20.0, 2.5)],
        }],
        (49, "S"),
    )

    assert payload["expansion_radius_m"] == 0.2
    assert payload["areas"][0]["name"] == "E1"
    assert payload["areas"][0]["boundary_utm"][0][0] < 0.0
    assert payload["bridges"][0]["centerline_utm"] == [[10.0, 2.5], [20.0, 2.5]]
    assert payload["bridges"][0]["corridors_utm"]
