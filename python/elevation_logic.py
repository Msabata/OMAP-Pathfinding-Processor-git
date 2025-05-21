# File: elevation_logic.py
import requests
import math
import json # For potential debugging output
import sys # For printing to stderr
from pyproj import CRS, Transformer
from pyproj.exceptions import ProjError

# --- Constants ---
# Define coordinate systems
CRS_LATLON = CRS("EPSG:4326")       # WGS84 Lat/Lon (for API)
CRS_PROJECTED = CRS("EPSG:5514")    # S-JTSK / Krovak (for internal calculations) - VERIFY THIS CODE

# API Configuration (Example: Open Elevation)
API_URL = "https://api.open-elevation.com/api/v1/lookup"
HEADERS = {'Accept': 'application/json', 'Content-Type': 'application/json'}
MAX_POINTS_PER_REQUEST = 100 # Adjust based on API limits and testing

# Safety Limit for Grid Dimensions
MAX_GRID_DIMENSION = 10000 # Prevent excessively large grids

def get_elevation_grid(
    # Anchor point
    known_lat, known_lon,           # WGS84 Lat/Lon corresponding to internal anchor
    known_internal_x, known_internal_y, # Internal anchor coords micrometers
    # Map definition
    raw_min_x_um, raw_min_y_um,     # Min bounds in micrometers micrometers
    raw_max_x_um, raw_max_y_um,     # Max bounds in micrometers micrometers
    map_scale,                      # Map scale denominator (e.g., 10000)
    # Query parameters
    desired_resolution_meters
    ):
    """
    Calculates projected bounds, generates a padded grid, queries elevation API,
    and returns results relative to the projected grid.
    Assumes raw bounds are in micrometers micrometers.
    Applies Y-inversion for offset calculation.
    Uses pyproj for accurate coordinate transformations.
    """
    print(f"[Python] Received request: AnchorLat={known_lat}, AnchorLon={known_lon}, "
          f"InternalAnchor=({known_internal_x},{known_internal_y}), MapScale={map_scale}, "
          f"BoundsUM=({raw_min_x_um},{raw_min_y_um})-({raw_max_x_um},{raw_max_y_um}), "
          f"Resolution={desired_resolution_meters}m")

    result_template = {
        'success': False, 'error_message': "", 'width': 0, 'height': 0,
        'origin_proj_x': 0.0, 'origin_proj_y': 0.0, # Origin is in Projected CRS
        'resolution_meters': 0.0, 'values': []
    }

    try:
        # --- 1. Setup & Input Validation ---
        if desired_resolution_meters <= 1e-9: # Use epsilon
            result_template['error_message'] = "Desired resolution must be positive and non-zero."
            return result_template
        if map_scale <= 0:
             result_template['error_message'] = "Map scale must be positive."
             return result_template

        # Correct scale factor: Meters per Internal Unit micrometers = map_scale / 1,000,000
        METERS_PER_INTERNAL_UNIT_UM = map_scale / 1_000_000.0
        print(f"[Python] Calculated meters per internal unit (micrometer): {METERS_PER_INTERNAL_UNIT_UM}")

        # --- POSSIBLE HANG POINT 1: Transformer Creation ---
        print("[Python Debug] Creating pyproj Transformers...")
        transformer_latlon_to_proj = Transformer.from_crs(CRS_LATLON, CRS_PROJECTED, always_xy=True)
        transformer_proj_to_latlon = Transformer.from_crs(CRS_PROJECTED, CRS_LATLON, always_xy=True)
        print("[Python Debug] Transformers created.")
        # --- END POSSIBLE HANG POINT 1 ---

        # --- 2. Convert Known Point ---
        # --- POSSIBLE HANG POINT 2: First Transform Call ---
        print("[Python Debug] Converting known anchor point...")
        known_proj_x, known_proj_y = transformer_latlon_to_proj.transform(known_lon, known_lat)
        print(f"[Python Debug] Anchor point conversion done.")
        # --- END POSSIBLE HANG POINT 2 ---

        # --- 3. Calculate Meter Offsets (Applying Y-Inversion) ---
        offset_min_x_m = (raw_min_x_um - known_internal_x) * METERS_PER_INTERNAL_UNIT_UM
        offset_min_y_m = (raw_min_y_um - known_internal_y) * METERS_PER_INTERNAL_UNIT_UM * -1.0 # Y-Inversion
        offset_max_x_m = (raw_max_x_um - known_internal_x) * METERS_PER_INTERNAL_UNIT_UM
        offset_max_y_m = (raw_max_y_um - known_internal_y) * METERS_PER_INTERNAL_UNIT_UM * -1.0 # Y-Inversion
        print(f"[Python] Calculated Meter Offsets from anchor: dX({offset_min_x_m:.1f}, {offset_max_x_m:.1f}), dY({offset_min_y_m:.1f}, {offset_max_y_m:.1f})")

        # --- 4. Calculate Exact Projected Bounds ---
        proj_x1 = known_proj_x + offset_min_x_m
        proj_x2 = known_proj_x + offset_max_x_m
        proj_y1 = known_proj_y + offset_min_y_m
        proj_y2 = known_proj_y + offset_max_y_m
        proj_min_x = min(proj_x1, proj_x2)
        proj_max_x = max(proj_x1, proj_x2)
        proj_min_y = min(proj_y1, proj_y2)
        proj_max_y = max(proj_y1, proj_y2)
        print(f"[Python] Calculated Projected Bounds (Map): X[{proj_min_x:.2f}, {proj_max_x:.2f}] Y[{proj_min_y:.2f}, {proj_max_y:.2f}]")

        # --- 5. Determine Grid Size (Inner) ---
        inner_width_m = proj_max_x - proj_min_x
        inner_height_m = proj_max_y - proj_min_y
        print(f"[Python Debug] Inner Dimensions (m): W={inner_width_m}, H={inner_height_m}")

        inner_grid_width = math.ceil(inner_width_m / desired_resolution_meters) if inner_width_m > 1e-9 else 1
        inner_grid_height = math.ceil(inner_height_m / desired_resolution_meters) if inner_height_m > 1e-9 else 1
        inner_grid_width = max(1, inner_grid_width)
        inner_grid_height = max(1, inner_grid_height)
        print(f"[Python Debug] Inner Grid Cells: W={inner_grid_width}, H={inner_grid_height}")

        # --- 6. Calculate Final Grid Dimensions & Origin (Padded) ---
        final_grid_width = inner_grid_width + 2
        final_grid_height = inner_grid_height + 2
        print(f"[Python Debug] Final Grid Cells: W={final_grid_width}, H={final_grid_height}")

        grid_origin_proj_x = proj_min_x - desired_resolution_meters
        grid_origin_proj_y = proj_min_y - desired_resolution_meters
        print(f"[Python Debug] Final Grid Origin Proj: X={grid_origin_proj_x}, Y={grid_origin_proj_y}")

        # --- 7. Generate Projected Query Points (Cell Centers) ---
        proj_query_points_x = []
        proj_query_points_y = []

        if final_grid_width <= 0 or final_grid_height <= 0 or final_grid_width > MAX_GRID_DIMENSION or final_grid_height > MAX_GRID_DIMENSION:
             raise ValueError(f"Invalid final grid dimensions calculated: {final_grid_width}x{final_grid_height}")

        total_points_expected = final_grid_width * final_grid_height
        print(f"[Python Debug] Total points expected: {total_points_expected} ({final_grid_width} x {final_grid_height})")

        print(f"[Python Debug] Generating {total_points_expected} query points...") # Added print
        for j in range(final_grid_height): # Rows (Y)
            y = grid_origin_proj_y + (j + 0.5) * desired_resolution_meters
            for i in range(final_grid_width): # Columns (X)
                x = grid_origin_proj_x + (i + 0.5) * desired_resolution_meters
                proj_query_points_x.append(x)
                proj_query_points_y.append(y)
        print(f"[Python Debug] Query point generation complete.") # Added print

        # --- 8. Convert Projected Query Points -> Lat/Lon for API ---
        # --- POSSIBLE HANG POINT 3: Batch Transform Call ---
        print(f"[Python Debug] Preparing bulk transform of {len(proj_query_points_x)} points...")
        lon_coords, lat_coords = transformer_proj_to_latlon.transform(proj_query_points_x, proj_query_points_y)
        print(f"[Python Debug] Bulk transform complete.")
        latlon_query_points = [{"latitude": lat, "longitude": lon} for lat, lon in zip(lat_coords, lon_coords)]
        print(f"[Python] Conversion to Lat/Lon complete.") # Renamed slightly
        # --- END POSSIBLE HANG POINT 3 ---

        # --- 9. Query Elevation API (Batched) ---
        all_elevations = [math.nan] * total_points_expected
        num_points_total = len(latlon_query_points)
        request_count = 0
        points_processed = 0

        print(f"[Python] Starting API queries in batches of {MAX_POINTS_PER_REQUEST}...")
        for i in range(0, num_points_total, MAX_POINTS_PER_REQUEST):
            batch_latlon = latlon_query_points[i : min(i + MAX_POINTS_PER_REQUEST, num_points_total)]
            if not batch_latlon: continue

            payload = {"locations": batch_latlon}
            request_count += 1
            batch_num_str = f"{request_count}/{math.ceil(num_points_total / MAX_POINTS_PER_REQUEST)}"

            print(f"[Python Batch {batch_num_str}] Preparing request ({len(batch_latlon)} points)...") # Renamed prefix

            try:
                print(f"[Python Batch {batch_num_str}] Executing requests.post...") # Renamed prefix
                # Keep verify=False for now, remove if SSL is fixed
                response = requests.post(API_URL, headers=HEADERS, json=payload, timeout=30, verify=False)
                print(f"[Python Batch {batch_num_str}] requests.post completed. Status: {response.status_code}") # Renamed prefix

                response.raise_for_status()
                results_json = response.json()
                print(f"[Python Batch {batch_num_str}] JSON response parsed.") # Renamed prefix

                if "results" not in results_json or len(results_json["results"]) != len(batch_latlon):
                    print(f"[Python Batch {batch_num_str}] Warning: API response length mismatch...") # Renamed prefix
                    for k in range(len(batch_latlon)): all_elevations[i+k] = math.nan # Fill batch
                    continue

                print(f"[Python Batch {batch_num_str}] Storing elevations...") # Renamed prefix
                for k, elev_data in enumerate(results_json["results"]):
                    point_index = i + k
                    if point_index < total_points_expected:
                         elevation = elev_data.get("elevation")
                         all_elevations[point_index] = float(elevation) if elevation is not None else math.nan
                    else: pass # Should not happen if logic is correct
                points_processed += len(batch_latlon)
                print(f"[Python Batch {batch_num_str}] Batch complete.") # Renamed prefix

            except requests.exceptions.Timeout:
                 print(f"[Python Batch {batch_num_str}] Error: API Request Timeout. Filling with NaN.")
                 for k in range(len(batch_latlon)): all_elevations[i+k] = math.nan
            except requests.exceptions.RequestException as e:
                print(f"[Python Batch {batch_num_str}] Error: API Request Failed: {e}. Filling with NaN.")
                for k in range(len(batch_latlon)): all_elevations[i+k] = math.nan
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
                 print(f"[Python Batch {batch_num_str}] Error: Parsing API Response: {e}. Filling with NaN.")
                 for k in range(len(batch_latlon)): all_elevations[i+k] = math.nan
            except Exception as e:
                 print(f"[Python Batch {batch_num_str}] Error: Unexpected Exception: {e}. Filling with NaN.")
                 for k in range(len(batch_latlon)): all_elevations[i+k] = math.nan

        print(f"[Python] Elevation fetch loop finished. Points processed: {points_processed}/{num_points_total}")

        # --- 10. Return Results ---
        final_result = result_template.copy()
        final_result.update({
            'success': True, # Mark success if loop finished (even if some batches failed w/ NaN)
            'width': final_grid_width,
            'height': final_grid_height,
            'origin_proj_x': grid_origin_proj_x,
            'origin_proj_y': grid_origin_proj_y,
            'resolution_meters': desired_resolution_meters,
            'values': all_elevations
        })
        # Optional: Check if TOO MANY NaNs indicate total failure
        nan_count = sum(math.isnan(v) for v in all_elevations)
        if nan_count > total_points_expected * 0.5: # Example: fail if > 50% are NaN
             print(f"[Python] Warning: High failure rate ({nan_count}/{total_points_expected} points are NaN). Marking overall result as failed.")
             final_result['success'] = False
             final_result['error_message'] = f"High failure rate during API calls ({nan_count}/{total_points_expected} points failed)."


        return final_result

    except ProjError as e:
        print(f"[Python] CRITICAL pyproj Error: {e}", file=sys.stderr)
        result_template['error_message'] = f"Coordinate Transformation Error: {e}"
        return result_template
    except ValueError as e: # Catch explicit ValueErrors (e.g., invalid grid dims)
        print(f"[Python] CRITICAL ValueError: {e}", file=sys.stderr)
        result_template['error_message'] = f"ValueError: {e}"
        return result_template
    except Exception as e:
        import traceback
        print(f"[Python] CRITICAL Unexpected Error: {e}", file=sys.stderr)
        # traceback.print_exc(file=sys.stderr) # Uncomment for full traceback
        result_template['error_message'] = f"Unexpected Python Error: {e}"
        return result_template

# --- Helper function for C++ to convert LatLon -> Projected ---
def convert_latlon_to_projected(lon, lat):
     """Converts a single WGS84 Lon/Lat point to the configured Projected CRS."""
     try:
         # Create transformer locally in case of threading issues with globals? Probably fine.
         transformer_latlon_to_proj = Transformer.from_crs(CRS_LATLON, CRS_PROJECTED, always_xy=True)
         proj_x, proj_y = transformer_latlon_to_proj.transform(lon, lat)
         return {'success': True, 'x': proj_x, 'y': proj_y}
     except Exception as e:
         return {'success': False, 'error': str(e)}

# --- Example usage for testing ---
if __name__ == '__main__':
     print("--- Running Python Script Test ---")
     # Use data similar to the debug output
     test_result = get_elevation_grid(
         known_lat=50.3406696, known_lon=12.78741839,
         known_internal_x=0.0, known_internal_y=0.0,
         raw_min_x_um=-50820.0, raw_min_y_um=72880.0,     # Pass micrometers  values
         raw_max_x_um=209920.0, raw_max_y_um=265830.0,    # Pass micrometers  values
         map_scale=10000.0,
         desired_resolution_meters=90.0
     )
     print("\n--- Python Test Result ---")
     if test_result:
        # Print results nicely, truncate long value list
        print(f"Success: {test_result['success']}")
        if not test_result['success']:
            print(f"Error: {test_result['error_message']}")
        else:
            print(f"Grid Dimensions: {test_result['width']} x {test_result['height']}")
            print(f"Resolution (m): {test_result['resolution_meters']}")
            print(f"Origin Projected ({CRS_PROJECTED.name}): X={test_result['origin_proj_x']:.2f}, Y={test_result['origin_proj_y']:.2f}")
            num_values = len(test_result['values'])
            print(f"Elevation Values: {num_values} points received.")
            if num_values > 0:
                 preview_count = min(num_values, 10)
                 print(f"  Preview (first {preview_count}): {test_result['values'][:preview_count]}")
     else:
         print("Function returned None (unexpected error)")

     # Test coordinate conversion helper
     print("\n--- Testing Coordinate Conversion ---")
     proj_coords = convert_latlon_to_projected(lon=12.78741839, lat=50.3406696)
     print(f"Lat/Lon (50.34, 12.78) -> Projected ({CRS_PROJECTED.name}): {proj_coords}")
