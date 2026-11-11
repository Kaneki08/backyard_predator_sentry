
box1 = (10, 20, 50, 80)
box2 = (10, 20, 50, 80)

def overlap_box(box1, box2):
    x1, y1, x2, y2 = box1
    x3, y3, x4, y4 = box2
    max_x1 = max(x1, x3)
    min_x2 = min(x2, x4)
    max_y1 = max(y1, y3)
    min_y2 = min(y2, y4)
    overlap = (max_x1, max_y1, min_x2, min_y2)
    print(overlap)
    return overlap



def area(overlap):
    if overlap == 0:
        return 0
    x1, y1, x2, y2 = overlap
    width = x2 - x1
    height = y2 - y1
    if x2 < x1 or y2 < y1:
        return 0
    return width * height


def iou(box1, box2):
    overlap = overlap_box(box1, box2)
    overlap_area = area(overlap)
    area1 = area(box1)
    area2 = area(box2)
    union_area = area1 + area2 - overlap_area
    if union_area == 0:
        return 0
    return overlap_area / union_area


if __name__ == "__main__":

    print(area(overlap_box(box1, box2)))
    print(iou(box1, box2))