from ml.eval.boxes import overlap_box, area, iou

def test_overlap_of_boxes():
    box1 = (0, 0, 10, 10)
    box2 = (5, 5, 15, 15)
    overlap = overlap_box(box1, box2)
    assert overlap == (5, 5, 10, 10)
    assert area(overlap) == 25

def test_area_of_backward_boxes():
    box1 = (10, 10, 20, 20)
    box2 = (0, 0, 5, 5)
    assert area(overlap_box(box1, box2)) == 0

def test_iou_box_examples():
    box1 = (10, 20, 50, 80)
    box2 = (30, 40, 70, 90)
    overlap = overlap_box(box1, box2)
    assert overlap == (30, 40, 50, 80)
    assert area(overlap) == 20 * 40
    assert iou(box1, box2) == (20 * 40) / (40 * 60 + 40 * 50 - 20 * 40) 

def test_iou_identical_boxes():
    box1 = (10, 20, 50, 80)
    box2 = (10, 20, 50, 80)
    overlap = overlap_box(box1, box2)
    assert overlap == box1
    assert area(overlap) == area(box1)
    assert iou(box1, box2) == 1

def test_iou_no_overlap():
    box1 = (10, 20, 50, 80)
    box2 = (60, 90, 100, 120)
    overlap = overlap_box(box1, box2)
    assert overlap == (60, 90, 50, 80)
    assert area(overlap) == 0
    assert iou(box1, box2) == 0

def test_iou_one_edge_overlap():
    box1 = (10, 20, 50, 80)
    box2 = (50, 20, 90, 80)
    overlap = overlap_box(box1, box2)
    assert overlap == (50, 20, 50, 80)
    assert area(overlap) == 0
    assert iou(box1, box2) == 0

def test_iou_4x_area_box():
    box1 = (10, 20, 50, 80) #area = 40 * 60x = 2400
    box2 = (10, 20, 100, 160) #area = 90 * 140 = 12600, 4x area than box1
    overlap = overlap_box(box1, box2)
    assert overlap == box1
    assert area(overlap) == area(box1)
    assert iou(box1, box2) == area(box1) / area(box2)