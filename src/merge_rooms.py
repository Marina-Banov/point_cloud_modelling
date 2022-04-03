import argparse
import json


def get_avg(arr, direction):
    direction = 0 if direction == "x" else 1
    return round(sum([el[direction] for el in arr]) / len(arr), 5)


def get_wall_summary(poly, cur_dir):
    new_dir = cur_dir
    walls = []
    wall = [poly[0]]
    for i in range(1, len(poly)):
        point1 = poly[i - 1]
        point2 = poly[i]
        if abs(point1[0] - point2[0]) <= 0.1:
            new_dir = "y"
        if abs(point1[1] - point2[1]) <= 0.1:
            new_dir = "x"
        if cur_dir != new_dir:
            cur_dir = new_dir
            walls.append((cur_dir, get_avg(wall, cur_dir), wall))
            wall = [point1, point2]
        else:
            wall.append(point2)
    cur_dir = "y" if cur_dir == "x" else "x"
    walls.append((cur_dir, get_avg(wall, cur_dir), wall))
    return walls


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f1", type=str, required=True, metavar="FILE_1",
        help="first .pcd file"
    )
    parser.add_argument(
        "-f2", type=str, required=True, metavar="FILE_2",
        help="second .pcd file"
    )
    parser.add_argument(
        "-p", type=str, required=True, metavar="HOLE_POS",
        help="position of the whole (%)"
    )
    parser.add_argument(
        "-w", type=str, required=True, metavar="HOLE_WIDTH",
        help="width of the hole (%)"
    )
    filename1 = parser.parse_args().f1
    filename2 = parser.parse_args().f2
    pos = parser.parse_args().p
    width = parser.parse_args().w

    negative_meshes = []

    with open(filename1) as f1:
        room1 = json.load(f1)
        poly1 = room1["definition"]["positivemeshes"][0]
        walls1 = get_wall_summary(poly1["polygon"], "y")
        for m in room1["definition"]["negativemeshes"]:
            negative_meshes.append(m)

    with open(filename2) as f2:
        room2 = json.load(f2)
        poly2 = room2["definition"]["positivemeshes"][0]
        walls2 = get_wall_summary(poly2["polygon"], "x")
        for m in room2["definition"]["negativemeshes"]:
            negative_meshes.append(m)

    poly = {
        "polygon": [],
        "bottom": min(poly1["bottom"], poly2["bottom"]),
        "top": max(poly1["top"], poly2["top"])
    }

    new_file_name = f'{room1["name"]}-{room2["name"]}'
    data = {
        "name": new_file_name,
        "definition": {
            "positivemeshes": [poly],
            "negativemeshes": negative_meshes
        }
    }
    with open(f'{new_file_name}.json', 'w') as outfile:
        json.dump(data, outfile, indent=4)


if __name__ == '__main__':
    main()
