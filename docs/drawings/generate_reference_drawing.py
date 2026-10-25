#!/usr/bin/env python3
"""Generate a vector A3 reference drawing from the C1 FreeCAD assemblies.

The extraction stage runs under FreeCADCmd and uses TechDraw.projectToSVG on
the saved C1 shapes.  The normal stage parses those SVG paths and places them
as vector line work in a three-page ReportLab PDF.  It intentionally never
writes the source FCStd files.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Sequence


ASSEMBLY_GROUPS = ["Fixed", "Swashplate", "Hub", "Blade1", "Blade2", "Blade3", "Blade4", "Blade5"]
HEAD_GROUPS = ["Fixed", "Swashplate", "Hub"]
ROOT_PARTS = [
    "Blade1HubArm", "Blade1OuterRace", "Blade1Elastomer", "Blade1Shims",
    "Blade1InnerRace", "Blade1Cuff", "Blade1PitchHorn", "Blade1PitchLink",
    "Blade1LowerBall", "Blade1UpperBall", "Blade1DamperMount", "Blade1DamperLug",
    "Blade1DamperBody", "Blade1DamperRod", "Blade1DA", "Blade1DB",
    "Blade1FoldClevis", "Blade1FoldTang", "Blade1FoldHinge", "Blade1FlightLock",
    "Blade1RootSparEnvelope",
]
VIEWS = {
    "top": (0.0, 0.0, 1.0),
    "front": (0.0, 1.0, 0.0),
    "right": (1.0, 0.0, 0.0),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def compound(doc, names):
    import Part  # type: ignore

    shapes = []
    for name in names:
        obj = doc.getObject(name)
        if obj is not None and hasattr(obj, "Shape") and not obj.Shape.isNull():
            shapes.append(obj.Shape)
    if not shapes:
        raise RuntimeError(f"No usable shapes found for {names}")
    return Part.makeCompound(shapes)


def shape_stats(shape) -> dict:
    bb = shape.BoundBox
    vertices = list(shape.Vertexes)
    max_radius = max((math.hypot(v.Point.x, v.Point.y) for v in vertices), default=0.0)
    return {
        "bbox_mm": [bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax],
        "size_mm": [bb.XLength, bb.YLength, bb.ZLength],
        "max_radius_mm": max_radius,
        "vertex_count": len(vertices),
        "solid_count": len(shape.Solids),
        "face_count": len(shape.Faces),
    }


def project_svg(shape, direction):
    import FreeCAD as App  # type: ignore
    import TechDraw  # type: ignore

    return TechDraw.projectToSVG(shape, App.Vector(*direction))


def extract_state(doc, state_name: str, svg_dir: Path) -> dict:
    svg_dir.mkdir(parents=True, exist_ok=True)
    groups = {
        "assembly": ASSEMBLY_GROUPS,
        "head": HEAD_GROUPS,
        "root": ROOT_PARTS,
    }
    shapes = {key: compound(doc, names) for key, names in groups.items()}
    views = {}
    for shape_key, shape in shapes.items():
        for view_name, direction in VIEWS.items():
            key = f"{shape_key}_{view_name}"
            svg = project_svg(shape, direction)
            svg_path = svg_dir / f"{state_name}_{key}.svg"
            svg_path.write_text(svg, encoding="utf-8")
            views[key] = {
                "svg_path": str(svg_path),
                "direction": list(direction),
                "path_count": svg.count("<path"),
            }
    features = {}
    for name in ["RotorRing", "HubCap", "Mast", "Blade1RootSparEnvelope"]:
        obj = doc.getObject(name)
        if obj is not None and hasattr(obj, "Shape"):
            features[name] = shape_stats(obj.Shape)
    return {
        "assembly": shape_stats(shapes["assembly"]),
        "head": shape_stats(shapes["head"]),
        "root": shape_stats(shapes["root"]),
        "features": features,
        "views": views,
    }


def extract(cad_neutral: Path, cad_folded: Path, json_out: Path, svg_dir: Path) -> None:
    import FreeCAD as App  # type: ignore

    json_out.parent.mkdir(parents=True, exist_ok=True)
    svg_dir.mkdir(parents=True, exist_ok=True)
    states = {}
    for state_name, cad_path in [("neutral", cad_neutral), ("folded", cad_folded)]:
        doc = App.openDocument(str(cad_path))
        try:
            doc.recompute()
            states[state_name] = extract_state(doc, state_name, svg_dir)
        finally:
            App.closeDocument(doc.Name)
    payload = {
        "generator": "generate_reference_drawing.py",
        "freecad": getattr(App, "Version", lambda: [])(),
        "source": {
            "neutral": str(cad_neutral.resolve()),
            "folded": str(cad_folded.resolve()),
            "neutral_sha256": sha256(cad_neutral),
            "folded_sha256": sha256(cad_folded),
        },
        "units": "mm",
        "projection": {
            "method": "FreeCAD TechDraw.projectToSVG",
            "directions": {name: list(direction) for name, direction in VIEWS.items()},
            "basis": "+X forward, +Y left, +Z up",
        },
        "states": states,
    }
    json_out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps({"json": str(json_out), "states": list(states)}, indent=2))


# SVG path reader.  TechDraw emits lines, circular arcs, quadratic curves and
# cubic curves.  Curves are sampled into short vector segments for ReportLab;
# this keeps the final PDF independent of a raster image or an SVG plug-in.
TOKEN_RE = re.compile(r"[AaCcHhLlMmQqSsTtVvZz]|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
COMMAND_ARGS = {"M": 2, "L": 2, "H": 1, "V": 1, "C": 6, "S": 4, "Q": 4, "T": 2, "A": 7}


def _arc_points(p0, rx, ry, rotation, large, sweep, p1):
    x0, y0 = p0
    x1, y1 = p1
    rx, ry = abs(rx), abs(ry)
    if rx == 0 or ry == 0 or (abs(x0 - x1) < 1e-12 and abs(y0 - y1) < 1e-12):
        return [p1]
    phi = math.radians(rotation % 360.0)
    cp, sp = math.cos(phi), math.sin(phi)
    dx, dy = (x0 - x1) / 2.0, (y0 - y1) / 2.0
    xp, yp = cp * dx + sp * dy, -sp * dx + cp * dy
    lam = (xp * xp) / (rx * rx) + (yp * yp) / (ry * ry)
    if lam > 1:
        scale = math.sqrt(lam)
        rx, ry = rx * scale, ry * scale
    sign = -1.0 if bool(large) == bool(sweep) else 1.0
    numerator = max(0.0, (rx * rx * ry * ry) - (rx * rx * yp * yp) - (ry * ry * xp * xp))
    denominator = rx * rx * yp * yp + ry * ry * xp * xp
    coef = sign * math.sqrt(numerator / denominator) if denominator else 0.0
    cxp, cyp = coef * (rx * yp / ry), coef * (-ry * xp / rx)
    cx, cy = cp * cxp - sp * cyp + (x0 + x1) / 2.0, sp * cxp + cp * cyp + (y0 + y1) / 2.0
    ux, uy = (xp - cxp) / rx, (yp - cyp) / ry
    vx, vy = (-xp - cxp) / rx, (-yp - cyp) / ry
    theta = math.atan2(uy, ux)
    delta = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
    if not sweep and delta > 0:
        delta -= 2 * math.pi
    elif sweep and delta < 0:
        delta += 2 * math.pi
    steps = max(2, min(64, int(abs(delta) * max(rx, ry) / 8.0) + 2))
    return [
        (
            cx + rx * math.cos(theta + delta * i / steps) * cp - ry * math.sin(theta + delta * i / steps) * sp,
            cy + rx * math.cos(theta + delta * i / steps) * sp + ry * math.sin(theta + delta * i / steps) * cp,
        )
        for i in range(1, steps + 1)
    ]


def svg_path_segments(path_d: str) -> list[list[tuple[float, float]]]:
    tokens = TOKEN_RE.findall(path_d)
    i = 0
    command = None
    current = (0.0, 0.0)
    start = current
    last_cubic = None
    last_quad = None
    result: list[list[tuple[float, float]]] = []
    segment: list[tuple[float, float]] | None = None

    def new_segment(point):
        nonlocal segment
        segment = [point]
        result.append(segment)

    def add(point):
        nonlocal current
        if segment is None:
            new_segment(current)
        if not segment or abs(segment[-1][0] - point[0]) > 1e-10 or abs(segment[-1][1] - point[1]) > 1e-10:
            segment.append(point)
        current = point

    def take(n):
        nonlocal i
        if i + n > len(tokens) or any(token.isalpha() for token in tokens[i:i + n]):
            return None
        values = [float(token) for token in tokens[i:i + n]]
        i += n
        return values

    while i < len(tokens):
        if tokens[i].isalpha():
            command = tokens[i]
            i += 1
            if command in "Zz":
                add(start)
                command = None
                last_cubic = last_quad = None
                continue
        if command is None:
            break
        upper = command.upper()
        relative = command.islower()
        values = take(COMMAND_ARGS[upper])
        if values is None:
            command = None
            continue
        if upper == "M":
            x, y = values
            if relative:
                x, y = x + current[0], y + current[1]
            current = (x, y)
            start = current
            new_segment(current)
            command = "l" if relative else "L"
            last_cubic = last_quad = None
        elif upper == "L":
            x, y = values
            add((x + current[0], y + current[1]) if relative else (x, y))
            last_cubic = last_quad = None
        elif upper == "H":
            x = values[0] + current[0] if relative else values[0]
            add((x, current[1]))
            last_cubic = last_quad = None
        elif upper == "V":
            y = values[0] + current[1] if relative else values[0]
            add((current[0], y))
            last_cubic = last_quad = None
        elif upper == "C":
            x1, y1, x2, y2, x, y = values
            if relative:
                x1, y1, x2, y2, x, y = x1 + current[0], y1 + current[1], x2 + current[0], y2 + current[1], x + current[0], y + current[1]
            p0 = current
            for step in range(1, 9):
                t = step / 8.0
                u = 1 - t
                add((u**3 * p0[0] + 3 * u * u * t * x1 + 3 * u * t * t * x2 + t**3 * x,
                     u**3 * p0[1] + 3 * u * u * t * y1 + 3 * u * t * t * y2 + t**3 * y))
            last_cubic, last_quad = ((x2, y2), (x, y)), None
        elif upper == "S":
            x2, y2, x, y = values
            if relative:
                x2, y2, x, y = x2 + current[0], y2 + current[1], x + current[0], y + current[1]
            x1, y1 = (2 * current[0] - last_cubic[0], 2 * current[1] - last_cubic[1]) if last_cubic else current
            p0 = current
            for step in range(1, 9):
                t = step / 8.0
                u = 1 - t
                add((u**3 * p0[0] + 3 * u * u * t * x1 + 3 * u * t * t * x2 + t**3 * x,
                     u**3 * p0[1] + 3 * u * u * t * y1 + 3 * u * t * t * y2 + t**3 * y))
            last_cubic, last_quad = ((x2, y2), (x, y)), None
        elif upper == "Q":
            x1, y1, x, y = values
            if relative:
                x1, y1, x, y = x1 + current[0], y1 + current[1], x + current[0], y + current[1]
            p0 = current
            for step in range(1, 9):
                t = step / 8.0
                u = 1 - t
                add((u * u * p0[0] + 2 * u * t * x1 + t * t * x,
                     u * u * p0[1] + 2 * u * t * y1 + t * t * y))
            last_quad, last_cubic = ((x1, y1), (x, y)), None
        elif upper == "T":
            x, y = values
            if relative:
                x, y = x + current[0], y + current[1]
            x1, y1 = (2 * current[0] - last_quad[0], 2 * current[1] - last_quad[1]) if last_quad else current
            p0 = current
            for step in range(1, 9):
                t = step / 8.0
                u = 1 - t
                add((u * u * p0[0] + 2 * u * t * x1 + t * t * x,
                     u * u * p0[1] + 2 * u * t * y1 + t * t * y))
            last_quad, last_cubic = ((x1, y1), (x, y)), None
        elif upper == "A":
            rx, ry, rotation, large, sweep, x, y = values
            if relative:
                x, y = x + current[0], y + current[1]
            for point in _arc_points(current, rx, ry, rotation, large, sweep, (x, y)):
                add(point)
            last_cubic = last_quad = None
    return [points for points in result if len(points) > 1]


def read_svg_segments(svg_path: Path) -> list[list[tuple[float, float]]]:
    svg = svg_path.read_text(encoding="utf-8")
    paths = re.findall(r'<path\b[^>]*\bd="([^"]+)"', svg)
    segments = []
    for path_d in paths:
        segments.extend(svg_path_segments(path_d))
    return segments


def transform_point(point, orientation: str):
    x, y = point
    if orientation == "top":
        return (x, -y)
    # TechDraw's front/right output has Z on the SVG horizontal axis. Rotate
    # those projected axes so the drawing reads with Z vertical on the page.
    return (y, x)


def transform_segments(segments, orientation: str):
    return [[transform_point(point, orientation) for point in segment] for segment in segments]


def bounds(segments):
    points = [point for segment in segments for point in segment]
    if not points:
        return (0.0, 1.0, 0.0, 1.0)
    return (min(p[0] for p in points), max(p[0] for p in points), min(p[1] for p in points), max(p[1] for p in points))


def mm_line(canvas, p1, p2):
    canvas.line(p1[0] * MM, p1[1] * MM, p2[0] * MM, p2[1] * MM)


def arrow(canvas, x, y, direction: int, vertical: bool = False):
    size, half = 2.2, 1.1
    path = canvas.beginPath()
    if vertical:
        path.moveTo(x * MM, y * MM)
        path.lineTo((x - half) * MM, (y + direction * size) * MM)
        path.lineTo((x + half) * MM, (y + direction * size) * MM)
    else:
        path.moveTo(x * MM, y * MM)
        path.lineTo((x + direction * size) * MM, (y - half) * MM)
        path.lineTo((x + direction * size) * MM, (y + half) * MM)
    path.close()
    canvas.drawPath(path, fill=1, stroke=0)


def draw_dim_h(canvas, x1, x2, y, ext_y1, ext_y2, label):
    canvas.saveState()
    canvas.setStrokeColor(BLUE)
    canvas.setFillColor(BLUE)
    canvas.setLineWidth(0.28 * MM)
    mm_line(canvas, (x1, ext_y1), (x1, y))
    mm_line(canvas, (x2, ext_y2), (x2, y))
    mm_line(canvas, (x1, y), (x2, y))
    arrow(canvas, x1, y, 1)
    arrow(canvas, x2, y, -1)
    canvas.setFont("Helvetica", 6.2)
    canvas.drawCentredString(((x1 + x2) / 2) * MM, (y + 2.2) * MM, label)
    canvas.restoreState()


def draw_dim_v(canvas, x, y1, y2, ext_x1, ext_x2, label):
    canvas.saveState()
    canvas.setStrokeColor(BLUE)
    canvas.setFillColor(BLUE)
    canvas.setLineWidth(0.28 * MM)
    mm_line(canvas, (ext_x1, y1), (x, y1))
    mm_line(canvas, (ext_x2, y2), (x, y2))
    mm_line(canvas, (x, y1), (x, y2))
    arrow(canvas, x, y1, 1, vertical=True)
    arrow(canvas, x, y2, -1, vertical=True)
    canvas.saveState()
    canvas.translate((x + 2.2) * MM, ((y1 + y2) / 2) * MM)
    canvas.rotate(90)
    canvas.setFont("Helvetica", 6.2)
    canvas.drawCentredString(0, 0, label)
    canvas.restoreState()
    canvas.restoreState()


def panel(canvas, box, title, subtitle=None):
    x0, y0, x1, y1 = box
    canvas.saveState()
    canvas.setFillColor(MIST)
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.35 * MM)
    canvas.roundRect(x0 * MM, y0 * MM, (x1 - x0) * MM, (y1 - y0) * MM, 1.5 * MM, fill=1, stroke=1)
    canvas.setFillColor(NAVY)
    canvas.setFont("Helvetica-Bold", 7.2)
    canvas.drawString((x0 + 4) * MM, (y1 - 9) * MM, title)
    if subtitle:
        canvas.setFillColor(SLATE)
        canvas.setFont("Helvetica", 5.6)
        canvas.drawRightString((x1 - 4) * MM, (y1 - 9) * MM, subtitle)
    canvas.restoreState()


def draw_view(canvas, svg_path, orientation, box, scale_denominator, title, subtitle=None):
    panel(canvas, box, title, subtitle)
    x0, y0, x1, y1 = box
    segments = transform_segments(read_svg_segments(Path(svg_path)), orientation)
    bx0, bx1, by0, by1 = bounds(segments)
    width, height = max(bx1 - bx0, 1e-6), max(by1 - by0, 1e-6)
    plot = (x0 + 4, y0 + 4, x1 - 4, y1 - 16)
    available_w, available_h = plot[2] - plot[0], plot[3] - plot[1]
    target_scale = 1.0 / scale_denominator
    actual_scale = min(target_scale, available_w / width, available_h / height)
    cx, cy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
    pcx, pcy = (plot[0] + plot[2]) / 2.0, (plot[1] + plot[3]) / 2.0

    def page(point):
        return (pcx + (point[0] - cx) * actual_scale, pcy + (point[1] - cy) * actual_scale)

    canvas.saveState()
    canvas.setStrokeColor(INK)
    canvas.setLineWidth(0.25 * MM)
    for segment in segments:
        first = page(segment[0])
        path = canvas.beginPath()
        path.moveTo(first[0] * MM, first[1] * MM)
        for point in segment[1:]:
            mapped = page(point)
            path.lineTo(mapped[0] * MM, mapped[1] * MM)
        canvas.drawPath(path, fill=0, stroke=1)
    canvas.restoreState()
    canvas.saveState()
    canvas.setFillColor(SLATE)
    canvas.setFont("Helvetica", 5.3)
    canvas.drawString((x0 + 4) * MM, (y0 + 4) * MM, f"REF SCALE 1:{actual_scale and round(1 / actual_scale):,}")
    canvas.restoreState()
    return {
        "page": page,
        "bounds": (bx0, bx1, by0, by1),
        "scale": actual_scale,
        "plot": plot,
    }


def header(canvas, page_no, title, subtitle):
    canvas.setFillColor(NAVY)
    canvas.setFont("Helvetica-Bold", 14)
    canvas.drawString(15 * MM, 282 * MM, title)
    canvas.setFillColor(SLATE)
    canvas.setFont("Helvetica", 7.3)
    canvas.drawString(15 * MM, 273 * MM, subtitle)
    canvas.setStrokeColor(BLUE)
    canvas.setLineWidth(0.6 * MM)
    canvas.line(15 * MM, 268 * MM, 405 * MM, 268 * MM)
    canvas.setFillColor(BLUE)
    canvas.setFont("Helvetica-Bold", 7.5)
    canvas.drawRightString(405 * MM, 282 * MM, f"SHEET {page_no:02d} / 03")


def footer(canvas, page_no):
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.3 * MM)
    canvas.line(15 * MM, 13 * MM, 405 * MM, 13 * MM)
    canvas.setFillColor(SLATE)
    canvas.setFont("Helvetica", 5.7)
    canvas.drawString(15 * MM, 8 * MM, "C1 CAD REFERENCE | NOT FOR MANUFACTURE | UN-TOLERANCED | ALL DIMENSIONS mm")
    canvas.drawRightString(405 * MM, 8 * MM, f"PAGE {page_no} / 3")


def note_box(canvas, x, y, w, h, title, lines):
    canvas.saveState()
    canvas.setFillColor(PALE)
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.3 * MM)
    canvas.roundRect(x * MM, y * MM, w * MM, h * MM, 1.5 * MM, fill=1, stroke=1)
    canvas.setFillColor(NAVY)
    canvas.setFont("Helvetica-Bold", 6.6)
    canvas.drawString((x + 4) * MM, (y + h - 9) * MM, title)
    canvas.setFillColor(INK)
    canvas.setFont("Helvetica", 5.6)
    line_y = y + h - 17
    for line in lines:
        canvas.drawString((x + 4) * MM, line_y * MM, line)
        line_y -= 7
    canvas.restoreState()


def compose_pdf(data_path: Path, output_path: Path) -> dict:
    from reportlab.lib.colors import HexColor
    from reportlab.lib.pagesizes import A3, landscape
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas as pdf_canvas

    global MM, INK, NAVY, BLUE, CYAN, PALE, SLATE, RULE, MIST
    MM = mm
    INK = HexColor("#172333")
    NAVY = HexColor("#002060")
    BLUE = HexColor("#004EA2")
    CYAN = HexColor("#38BEE2")
    PALE = HexColor("#D7F2F9")
    SLATE = HexColor("#5D6A76")
    RULE = HexColor("#B0B8DD")
    MIST = HexColor("#F3F6F8")

    data = json.loads(data_path.read_text(encoding="utf-8"))
    neutral = data["states"]["neutral"]
    folded = data["states"]["folded"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pdf = pdf_canvas.Canvas(str(output_path), pagesize=landscape(A3), pageCompression=1)
    pdf.setTitle("Rotor C1 Reference Drawing")
    pdf.setAuthor("FreeCAD TechDraw projection from C1 CAD")

    # Sheet 1: complete neutral assembly.
    header(pdf, 1, "ROTOR C1 / 5-BLADE MAIN ROTOR", "Neutral flight configuration - orthographic reference views")
    top = draw_view(pdf, neutral["views"]["assembly_top"]["svg_path"], "top", (15, 72, 225, 266), 100, "A  TOP", "LOOK DIRECTION +Z")
    front = draw_view(pdf, neutral["views"]["assembly_front"]["svg_path"], "front", (235, 169, 405, 266), 120, "B  FRONT", "LOOK DIRECTION +Y")
    right = draw_view(pdf, neutral["views"]["assembly_right"]["svg_path"], "right", (235, 72, 405, 163), 120, "C  RIGHT", "LOOK DIRECTION +X")
    nominal = 9300.0
    # The nominal diameter is a design circle, not the assembly bounding-box
    # width.  Show it as a dashed construction circle plus a plain callout.
    nominal_cx, nominal_cy = top["page"]((0.0, 0.0))
    pdf.saveState()
    pdf.setStrokeColor(BLUE)
    pdf.setLineWidth(0.25 * MM)
    pdf.setDash(2 * MM, 1 * MM)
    pdf.circle(nominal_cx * MM, nominal_cy * MM, nominal * top["scale"] * MM, fill=0, stroke=1)
    pdf.setDash()
    pdf.setFillColor(BLUE)
    pdf.setFont("Helvetica", 6.2)
    pdf.drawCentredString(120 * MM, 67 * MM, "NOMINAL ROTOR DIA 18,600 (DESIGN R = 9,300)")
    pdf.restoreState()
    fbb = neutral["assembly"]["bbox_mm"]
    fy1 = front["page"]((front["bounds"][0], front["bounds"][2]))[1]
    fy2 = front["page"]((front["bounds"][0], front["bounds"][3]))[1]
    vx = front["page"]((front["bounds"][1], front["bounds"][0]))[0] + 5
    draw_dim_v(pdf, min(vx, 404), fy1, fy2, front["page"]((front["bounds"][1], front["bounds"][2]))[0], front["page"]((front["bounds"][1], front["bounds"][3]))[0], f"HEAD ENVELOPE Z {fbb[5] - fbb[4]:,.0f}")
    note_box(pdf, 235, 19, 170, 45, "MEASURED FROM SOURCE CAD", [
        f"Neutral assembly XY envelope: {fbb[1] - fbb[0]:,.1f} x {fbb[3] - fbb[2]:,.1f}",
        f"Maximum radial envelope: DIA {neutral['assembly']['max_radius_mm'] * 2:,.1f} (includes hardware)",
        "Nominal diameter is the design radius value (R 9,300); envelope is not a tolerance.",
        "Coordinates: +X forward / +Y left / +Z up; dimensions are CAD bounding/envelope values.",
    ])
    note_box(pdf, 15, 19, 210, 45, "DRAWING STATUS", [
        "REFERENCE DRAWING - NOT A MANUFACTURING DRAWING",
        "Neutral state loaded from rotor-neutral.FCStd; linework is TechDraw projected geometry.",
        "Views are labelled by direction vector; layout is arranged for readability, not ISO projection spacing.",
        "No tolerances, materials, loads, bearing fits, or airworthiness claims are implied.",
    ])
    footer(pdf, 1)
    pdf.showPage()

    # Sheet 2: rotor head and root module details.
    header(pdf, 2, "ROTOR HEAD / ROOT DETAIL", "Neutral state - enlarged views for interface and fold-reference geometry")
    htop = draw_view(pdf, neutral["views"]["head_top"]["svg_path"], "top", (15, 116, 210, 266), 20, "D  HEAD TOP")
    hfront = draw_view(pdf, neutral["views"]["head_front"]["svg_path"], "front", (220, 116, 405, 266), 20, "E  HEAD FRONT")
    rtop = draw_view(pdf, neutral["views"]["root_top"]["svg_path"], "top", (15, 39, 210, 108), 10, "F  BLADE 1 ROOT / FOLD")
    rfront = draw_view(pdf, neutral["views"]["root_front"]["svg_path"], "front", (220, 39, 405, 108), 10, "G  ROOT FRONT")
    rotor_ring = neutral["features"]["RotorRing"]["size_mm"]
    hub_cap = neutral["features"]["HubCap"]["size_mm"] if "HubCap" in neutral["features"] else [800, 800, 45]
    mast = neutral["features"]["Mast"]["size_mm"] if "Mast" in neutral["features"] else [320, 320, 2440]
    root_bb = neutral["features"]["Blade1RootSparEnvelope"]["size_mm"]
    draw_dim_h(pdf, *[htop["page"]((p, 0.0))[0] for p in (-740.0, 740.0)], 111, 119, 119, f"ROTOR RING DIA {rotor_ring[0]:,.0f}")
    hy1 = hfront["page"]((0.0, hfront["bounds"][2]))[1]
    hy2 = hfront["page"]((0.0, hfront["bounds"][3]))[1]
    draw_dim_v(pdf, 402, hy1, hy2, 394, 394, f"HEAD Z ENVELOPE {neutral['head']['size_mm'][2]:,.0f}")
    note_box(pdf, 15, 16, 390, 20, "DETAIL DIMENSIONS", [
        f"Rotor ring envelope DIA {rotor_ring[0]:,.0f} | Hub cap DIA {hub_cap[0]:,.0f} | Mast envelope DIA {mast[0]:,.0f} | Root spar envelope {root_bb[0]:,.0f} x {root_bb[1]:,.0f} x {root_bb[2]:,.0f}",
    ])
    footer(pdf, 2)
    pdf.showPage()

    # Sheet 3: folded/parked top reference.
    header(pdf, 3, "FOLDED / PARKED PLAN REFERENCE", "Folded state - envelope and storage-width evidence from rotor-folded.FCStd")
    ftop = draw_view(pdf, folded["views"]["assembly_top"]["svg_path"], "top", (15, 119, 405, 266), 75, "H  FOLDED TOP", "LOOK DIRECTION +Z")
    ffront = draw_view(pdf, folded["views"]["assembly_front"]["svg_path"], "front", (15, 39, 205, 111), 100, "I  FOLDED FRONT")
    fright = draw_view(pdf, folded["views"]["assembly_right"]["svg_path"], "right", (220, 39, 405, 111), 100, "J  FOLDED RIGHT")
    fbb = folded["assembly"]["bbox_mm"]
    fy1 = ftop["page"]((0.0, fbb[2]))[1]
    fy2 = ftop["page"]((0.0, fbb[3]))[1]
    draw_dim_v(pdf, 402, fy1, fy2, 394, 394, f"FOLDED Y PROJECTION {fbb[3] - fbb[2]:,.1f}")
    note_box(pdf, 15, 16, 390, 20, "FOLDED STATE NOTES", [
        f"Measured folded assembly envelope: X {fbb[1] - fbb[0]:,.1f} x Y {fbb[3] - fbb[2]:,.1f} x Z {fbb[5] - fbb[4]:,.1f} mm; width is a fixed-axis Y projection.",
    ])
    footer(pdf, 3)
    pdf.save()
    return {
        "output_pdf": str(output_path.resolve()),
        "neutral_nominal_diameter_mm": 2 * nominal,
        "neutral_envelope_diameter_mm": neutral["assembly"]["max_radius_mm"] * 2,
        "folded_width_mm": fbb[3] - fbb[2],
        "pages": 3,
    }


def find_freecadcmd(explicit: str | None) -> str:
    candidates = []
    if explicit:
        candidates.append(explicit)
    candidates.extend([
        os.environ.get("FREECADCMD", ""),
        shutil.which("FreeCADCmd.exe") or "",
        shutil.which("freecadcmd") or "",
    ])
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            return candidate
    raise FileNotFoundError("FreeCADCmd.exe not found; pass --freecadcmd explicitly")


def run_extraction(args, json_out: Path, svg_dir: Path):
    # FreeCADCmd treats dashed arguments as its own options.  A tiny generated
    # launcher keeps the user-facing command portable without relying on that
    # version-specific argument parser behavior.
    stage = Path(args.work_dir) / "_extract_freecad.py"
    stage.parent.mkdir(parents=True, exist_ok=True)
    stage.write_text(
        "import sys\n"
        f"sys.path.insert(0, {str(Path(__file__).resolve().parent)!r})\n"
        "from pathlib import Path\n"
        "from generate_reference_drawing import extract\n"
        f"extract(Path({str(Path(args.cad_neutral).resolve())!r}), Path({str(Path(args.cad_folded).resolve())!r}), Path({str(json_out.resolve())!r}), Path({str(svg_dir.resolve())!r}))\n",
        encoding="utf-8",
    )
    command = [find_freecadcmd(args.freecadcmd), str(stage.resolve())]
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    if completed.returncode:
        raise RuntimeError(f"FreeCAD extraction failed ({completed.returncode}):\n{completed.stdout}\n{completed.stderr}")
    if completed.stdout:
        print(completed.stdout)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-neutral", required=True)
    parser.add_argument("--cad-folded", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--freecadcmd")
    parser.add_argument("--freecad-stage", action="store_true")
    parser.add_argument("--json-out")
    parser.add_argument("--svg-dir")
    args = parser.parse_args(argv)
    work_dir = Path(args.work_dir)
    json_out = Path(args.json_out) if args.json_out else work_dir / "geometry.json"
    svg_dir = Path(args.svg_dir) if args.svg_dir else work_dir / "svg"
    if args.freecad_stage:
        extract(Path(args.cad_neutral), Path(args.cad_folded), json_out, svg_dir)
        return 0
    run_extraction(args, json_out, svg_dir)
    result = compose_pdf(json_out, Path(args.output))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
