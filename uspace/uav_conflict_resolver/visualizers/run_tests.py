"""
Script to run all conflict detection tests.
"""

from test_flight_plans import run_all_tests


def detect_conflict_with_obb(fp1, fp2):
    """
    Detect conflicts between two flight plans using Oriented Bounding Boxes (OBB).
    
    Returns True if conflict is detected, False otherwise.
    """
    # Generate swept boxes for both flight plans
    boxes1 = fp1.generate_swept_boxes_obb(interval=0.5)
    boxes2 = fp2.generate_swept_boxes_obb(interval=0.5)
    
    # Check each box from fp1 against each box from fp2
    for box1 in boxes1:
        for box2 in boxes2:
            if box1.collides_with(box2):
                return True
    
    return False


def detect_conflict_with_aabb(fp1, fp2):
    """
    Detect conflicts between two flight plans using Axis-Aligned Bounding Boxes (AABB).
    
    Returns True if conflict is detected, False otherwise.
    """
    # Generate swept boxes for both flight plans
    boxes1 = fp1.generate_swept_boxes_aabb(interval=0.5)
    boxes2 = fp2.generate_swept_boxes_aabb(interval=0.5)
    
    # Check each box from fp1 against each box from fp2
    for box1 in boxes1:
        for box2 in boxes2:
            if box1.collides_with(box2):
                return True
    
    return False


if __name__ == "__main__":
    import sys
    
    print("\n" + "="*80)
    print("RUNNING CONFLICT DETECTION TEST SUITE")
    print("="*80 + "\n")
    
    # Choose detector based on command line argument
    detector_choice = "obb"  # default
    if len(sys.argv) > 1:
        detector_choice = sys.argv[1].lower()
    
    if detector_choice == "aabb":
        print("Using AABB collision detector...\n")
        results = run_all_tests(detect_conflict_with_aabb)
    else:
        print("Using OBB collision detector...\n")
        results = run_all_tests(detect_conflict_with_obb)
    
    print("\nDone! Check the results above.")
