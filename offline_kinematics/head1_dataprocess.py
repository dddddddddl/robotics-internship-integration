"""Process a supplied head metrology workbook; no measurement data are bundled."""
import argparse
from measurement_processing import print_head_results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workbook", help="Path to the original head measurement workbook")
    parser.add_argument("--sheet", default="Sheet1")
    args = parser.parse_args()
    print_head_results(args.workbook, args.sheet)
