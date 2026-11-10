"""Build the comparative MAN10T performance report from measured artifacts."""

import argparse
from collections import defaultdict
import json
from math import hypot, isfinite, sqrt
from pathlib import Path
import re
import statistics as stats
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT, TA_RIGHT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import Paragraph, Table, TableStyle


NAVY = colors.HexColor("#172C40")
TEAL = colors.HexColor("#006E66")
BLUE = colors.HexColor("#476C9D")
MUTED = colors.HexColor("#45596C")
GRID = colors.HexColor("#D8E2EA")
PAPER = colors.HexColor("#F2F6FA")
PALE = colors.HexColor("#E5F3EF")
ORANGE = colors.HexColor("#88521E")
AMBER = colors.HexColor("#FCF2E3")
ROUTE_COLORS = [colors.HexColor(c) for c in (
    "#285A9F", "#A43C3A", "#3E7851", "#744498", "#B36522",
    "#007877", "#A03569", "#55712A", "#765542", "#665D90")]
W, H = A4
LEFT = 44
CONTENT = W - 2 * LEFT
TOP = 47
OUT = Path("output/pdf/informe-profiling-man10t-rimls-cpp.pdf")
BASE = Path("../FusionForce/output/man10t-work")
DATA = Path("output/man10t-profiling-cpp")


def fmt(value, digits=2):
    if value is None:
        return "n/d"
    return f"{value:,.{digits}f}".translate(str.maketrans(",.", ".,"))


def unit(value, suffix, digits=2):
    return "n/d" if value is None else f"{fmt(value, digits)} {suffix}"


def sample_mean(rows, key):
    values = [r.get(key) for r in rows]
    valid = [v for v in values if isinstance(v, (float, int)) and isfinite(v)]
    return stats.mean(valid) if valid else None


def gpu_count(rows):
    return sum(isinstance(row.get("gpu_mean_w"), (float, int)) for row in rows)


def configure_fonts():
    windows = Path("C:/Windows/Fonts")
    font_paths = [windows / name for name in ("segoeui.ttf", "segoeuib.ttf", "consola.ttf")]
    if all(path.exists() for path in font_paths):
        for name, path in zip(("Text", "Bold", "Mono"), font_paths):
            pdfmetrics.registerFont(TTFont(name, str(path)))
        return "Text", "Bold", "Mono"
    return "Helvetica", "Helvetica-Bold", "Courier"


FONT, BOLD, MONO = configure_fonts()


class Report:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.doc = canvas.Canvas(str(path), pagesize=A4)
        self.doc.setTitle("FusionForce | Perfil comparativo MAN10T en rimls.ply")
        self.doc.setAuthor("FusionForce")
        self.page_number = 0

    def text(self, x, top, value, size=10, fill=NAVY, font=FONT, align="left"):
        self.doc.setFillColor(fill)
        self.doc.setFont(font, size)
        draw = self.doc.drawRightString if align == "right" else self.doc.drawString
        draw(x, H - top, str(value))

    def para(self, top, value, size=10, fill=MUTED, x=LEFT, width=CONTENT, font=FONT):
        style = ParagraphStyle("text", fontName=font, fontSize=size, leading=size * 1.44,
                               textColor=fill, spaceBefore=0, spaceAfter=0)
        paragraph = Paragraph(escape(value), style)
        _, height = paragraph.wrap(width, H)
        paragraph.drawOn(self.doc, x, H - top - height)
        return height

    def rect(self, x, top, width, height, fill):
        self.doc.setFillColor(fill)
        self.doc.rect(x, H - top - height, width, height, fill=1, stroke=0)

    def line(self, x0, y0, x1, y1, stroke=GRID, width=.6, dash=None):
        self.doc.setStrokeColor(stroke)
        self.doc.setLineWidth(width)
        self.doc.setDash(dash or [])
        self.doc.line(x0, H - y0, x1, H - y1)
        self.doc.setDash([])

    def section(self, top, title):
        self.text(LEFT, top, title, 13, NAVY, BOLD)

    def page(self, section, title, intro):
        self.page_number += 1
        self.rect(0, 0, W, 8, TEAL)
        self.text(LEFT, TOP, f"FUSIONFORCE   /   MAN10T   /   {section.upper()}", 9.2, TEAL, BOLD)
        self.text(LEFT, TOP + 37, title, 27, NAVY, BOLD)
        self.para(TOP + 53, intro, 11)
        self.line(LEFT, 802, W - LEFT, 802)
        self.text(LEFT, 821, "Perfil comparativo  /  rimls.ply", 8.5, MUTED)
        self.text(W - LEFT, 821, f"{self.page_number:02d} / 05", 8.5, MUTED, BOLD, "right")

    def card(self, index, value, label):
        width = (CONTENT - 20) / 3
        x = LEFT + index * (width + 10)
        self.rect(x, 152, width, 72, PALE if index == 2 else PAPER)
        self.text(x + 12, 185, value, 25, TEAL if index == 2 else NAVY, BOLD)
        self.para(191, label, 8.6, MUTED, x + 12, width - 24)

    def box(self, top, title, body, fill=PAPER):
        inner_height = self.para_measure(body, 9.8, CONTENT - 26)
        self.rect(LEFT, top, CONTENT, 34 + inner_height, fill)
        self.text(LEFT + 13, top + 18, title, 10, TEAL if fill != AMBER else ORANGE, BOLD)
        self.para(top + 26, body, 9.8, MUTED, LEFT + 13, CONTENT - 26)
        return top + 34 + inner_height

    @staticmethod
    def para_measure(value, size, width):
        style = ParagraphStyle("measure", fontName=FONT, fontSize=size, leading=size * 1.44)
        return Paragraph(escape(value), style).wrap(width, H)[1]

    def table(self, top, labels, rows, widths, numeric=(), size=9.1, padding=5):
        def cell(text, header, number):
            style = ParagraphStyle("cell", fontName=BOLD if header else FONT,
                                   fontSize=size, leading=size * 1.25,
                                   textColor=colors.white if header else NAVY,
                                   alignment=TA_RIGHT if number else TA_LEFT)
            return Paragraph(escape(str(text)).replace("\n", "<br/>"), style)

        table_rows = [[cell(value, i == 0, j in numeric) for j, value in enumerate(row)]
                      for i, row in enumerate([labels, *rows])]
        table = Table(table_rows, colWidths=widths, hAlign="LEFT")
        table.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), NAVY),
            ("ROWBACKGROUNDS", (0, 1), (-1, -1), [PAPER, colors.white]),
            ("LINEBELOW", (0, -1), (-1, -1), .7, GRID),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("LEFTPADDING", (0, 0), (-1, -1), 8),
            ("RIGHTPADDING", (0, 0), (-1, -1), 8),
            ("TOPPADDING", (0, 0), (-1, -1), padding),
            ("BOTTOMPADDING", (0, 0), (-1, -1), padding),
        ]))
        _, height = table.wrap(CONTENT, H)
        table.drawOn(self.doc, LEFT, H - top - height)
        return top + height

    def legend_mark(self, x, top, color, label, dashed=False):
        self.line(x, top - 3, x + 17, top - 3, color, 2, [3, 2] if dashed else None)
        self.text(x + 22, top, label, 8.7, MUTED)

    def end(self):
        self.doc.showPage()


def horizontal_chart(report, rows):
    left = LEFT + 35
    right = W - LEFT - 53
    top, step = 285, 31
    maximum = max(max(old["wall_s"], new["wall_s"]) for old, new in rows)
    scale = (right - left) / (maximum * 1.12)
    tick_step = 2 if maximum >= 6 else 1
    for tick in range(0, int(maximum * 1.12) + 1, tick_step):
        x = left + tick * scale
        report.line(x, top - 5, x, top + 10 * step - 1)
        report.text(x, top + 10 * step + 14, str(tick), 8.2, MUTED)
    for i, (old, new) in enumerate(rows):
        y = top + step * i
        report.text(LEFT, y + 14, old["id"], 9, NAVY, BOLD)
        for row, offset, color in [(old, 0, BLUE), (new, 11, TEAL)]:
            value = row["wall_s"]
            width = value * scale
            report.rect(left, y + offset, width, 7.4, color)
            report.text(left + width + 5, y + offset + 7.2, fmt(value, 3 if offset else 2),
                        8.1, color)
    report.text(right, top + 10 * step + 30, "Tiempo real por ruta (s)", 8.2, MUTED, align="right")


def positions(row):
    return [(p[0], p[1]) for p in row.get("xy", [])]


def route_chart(report, rows, reference):
    paths = [positions(row) for pair in rows for row in pair]
    paths += list(reference.values())
    points = [point for path in paths for point in path]
    if not points:
        report.para(177, "El registro no contiene coordenadas XY.")
        return
    x0, x1 = min(p[0] for p in points) - 5, max(p[0] for p in points) + 5
    y0, y1 = min(p[1] for p in points) - 5, max(p[1] for p in points) + 5
    plot_left, plot_top, plot_w, plot_h = LEFT + 35, 177, CONTENT - 55, 285
    scale = min(plot_w / (x1 - x0), plot_h / (y1 - y0))
    x_offset = plot_left + (plot_w - (x1 - x0) * scale) / 2
    y_offset = plot_top + (plot_h - (y1 - y0) * scale) / 2

    def project(point):
        return x_offset + (point[0] - x0) * scale, y_offset + (y1 - point[1]) * scale

    minx, miny = project((x0, y1))
    maxx, maxy = project((x1, y0))
    report.rect(minx, miny, maxx - minx, maxy - miny, PAPER)
    for x in range(-150, 151, 10):
        if x0 < x < x1:
            xp, _ = project((x, y0))
            report.line(xp, miny, xp, maxy)
            report.text(xp + 4, maxy + 13, str(x), 7.7, MUTED, align="right")
    for y in range(-150, 151, 10):
        if y0 < y < y1:
            _, yp = project((x0, y))
            report.line(minx, yp, maxx, yp)
            report.text(minx - 5, yp + 3, str(y), 7.7, MUTED, align="right")
    report.text(maxx + 6, maxy + 13, "X (m)", 8.3, MUTED)
    report.text(minx - 10, miny + 5, "Y (m)", 8.3, MUTED, align="right")
    for guide in reference.values():
        draw_path(report, project, guide, colors.HexColor("#8495A5"), .6, [1, 2.2])
    for i, (old, new) in enumerate(rows):
        color = ROUTE_COLORS[i]
        before, after = positions(old), positions(new)
        draw_path(report, project, before, color, 1.15, [3, 2])
        draw_path(report, project, after, color, 1.9)
        if after:
            x, y = project(after[-1])
            report.doc.setFillColor(color)
            report.doc.circle(x, H - y, 2.1, fill=1, stroke=0)
        start = after[0] if after else before[0] if before else None
        if start:
            x, y = project(start)
            report.text(x + 4, y - 4, old["id"], 7.7, color, BOLD)


def draw_path(report, project, points, color, width, dash=None):
    if not points:
        return
    path = report.doc.beginPath()
    for i, point in enumerate(points):
        x, y = project(point)
        (path.moveTo if i == 0 else path.lineTo)(x, H - y)
    report.doc.setLineWidth(width)
    report.doc.setStrokeColor(color)
    report.doc.setDash(dash or [])
    report.doc.drawPath(path)
    report.doc.setDash([])


def path_differences(old, new):
    before, after = positions(old), positions(new)
    if not before or not after:
        return None, None
    end = hypot(before[-1][0] - after[-1][0], before[-1][1] - after[-1][1])
    if len(before) != len(after) or old["simulated_s"] != new["simulated_s"]:
        return None, end
    sq = [(p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 for p, q in zip(before, after)]
    return sqrt(stats.mean(sq)), end


GPROF_ROW = re.compile(r"^\s*(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(.*)$")
GPROF_CALLS = re.compile(r"^(\d+)\s+\d+(?:\.\d+)?\s+\d+(?:\.\d+)?\s+(.*)$")


def read_gprof(paths):
    functions = defaultdict(float)
    call_counts = defaultdict(int)
    intervals = set()
    for path in paths:
        started = False
        for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
            if "Each sample counts as" in line:
                interval = re.search(r"(\d+\.\d+) seconds", line)
                if interval:
                    intervals.add(float(interval[1]))
            if "cumulative" in line and "self" in line:
                started = True
                continue
            if not started:
                continue
            match = GPROF_ROW.match(line)
            if match:
                calls = GPROF_CALLS.match(match[4])
                name = calls[2] if calls else match[4]
                functions[name] += float(match[3])
                if calls:
                    call_counts[name] += int(calls[1])
            elif line.lstrip().startswith("%") or line.startswith("Call graph"):
                break
    return sorted(functions.items(), key=lambda row: -row[1]), intervals, call_counts


NATIVE_NOTES = {
    "CloudStore::neighbors": "Busca vecinos de la nube y resuelve la superficie superior en XY.",
    "upper_xy": "Consulta la superficie superior de la nube por posición XY.",
    "nearest_xy": "Busca observaciones próximas en el índice espacial.",
    "TerrainPatch::search": "Recorre el índice espacial del parche para hallar el punto más próximo.",
    "__merge_sort_with_buffer": "Ordena candidatos de la nube para agrupar consultas espaciales.",
    "TerrainPatch::partition": "Construye el índice espacial de los puntos del parche.",
    "TruckStepper::step": "Combina terreno, geometría de contacto y avance de la dinámica.",
    "TerrainPatch::covered_point": "Comprueba la cobertura de la nube bajo cada contacto.",
    "TerrainPatch::update": "Actualiza y reutiliza el parche físico de terreno.",
    "TerrainPatch::sample": "Consulta la altura y la normal de la malla bajo un contacto.",
    "HeightGrid::sample": "Interpola la altura y la normal entre celdas de la malla.",
    "rigid::step": "Calcula contactos, fuerzas e integración del cuerpo rígido.",
    "transform_truck": "Transforma geometría, empujes e inercia del camión.",
    "rotation_matrix": "Calcula una matriz de rotación a partir de un cuaternión.",
    "StabilityMonitor::update": "Actualiza los índices de estabilidad por paso.",
    "material_at": "Asigna propiedades del suelo a cada punto de contacto.",
    "simulate_truck": "Orquesta la física y los resultados de cada recorrido.",
    "simulation_result_json": "Construye los resultados de reproducción en memoria.",
    "sqlite": "Trabaja en el índice persistente de la nube.",
    "basic_json": "Gestiona valores de la representación JSON en memoria.",
    "std::": "Símbolo de biblioteca estándar presente en el perfil.",
}
PYTHON_NOTES = {
    "upper_xy": "Selecciona los puntos de mayor cota para las consultas XY.",
    "evaluate": "Orquesta el bucle del motor, terreno y resultados.",
    "q_to_R": "Convierte cuaterniones en matrices de rotación.",
    "reduce": "Reduce arreglos NumPy mediante operaciones vectorizadas.",
    "assemble_and_transform_robot": "Transforma geometría, empujes e inercia.",
    "norm": "Calcula normas vectoriales.",
    "visual_frame": "Convierte arreglos en listas para la reproducción.",
}


def function_name(symbol):
    symbol = symbol.strip()
    if symbol.startswith("non-virtual thunk to "):
        symbol = symbol.removeprefix("non-virtual thunk to ")
    if symbol.startswith("fusionforce::"):
        symbol = symbol.removeprefix("fusionforce::")
    if "basic_json" in symbol and not symbol.startswith(("Cloud", "Terrain", "Truck")):
        return "JSON / " + ("destrucción" if "destroy" in symbol else "operaciones de biblioteca")
    if "__merge_sort_with_buffer" in symbol:
        return "std::__merge_sort_with_buffer"
    symbol = symbol.split("(", 1)[0].strip()
    return symbol if len(symbol) < 50 else symbol[:46] + "..."


def describe(symbol, explanations):
    return next((note for part, note in explanations.items() if part in symbol),
                "Símbolo registrado en el perfil bruto.")


def num_completed(rows):
    return sum(row.get("status") == "complete" and row.get("samples") == 1001 and
               abs(row.get("simulated_s", 0) - 10.0) < 1e-9 for row in rows)


def profile_total(run):
    return sum(row["wall_s"] for row in run.get("profiled", []))


def percent(value):
    return unit(value, "%", 1)


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def make_report(baseline, data, output, gprof_paths):
    old = load_json(baseline / "metrics.json")
    new = load_json(data / "metrics.json")
    route_path = data / "routes.json"
    if not route_path.exists():
        route_path = baseline / "routes.json"
    fixture = load_json(route_path)
    old_rows, new_rows = old["clean"], new["clean"]
    order = [f"R{i:02d}" for i in range(1, 11)]
    if [r["id"] for r in old_rows] != order or [r["id"] for r in new_rows] != order:
        raise ValueError("Se requieren las medidas R01 a R10 de ambas versiones")
    pairs = list(zip(old_rows, new_rows))
    guides = {route["id"]: [(p["x"], p["y"]) for p in route["payload"]["route"]]
              for route in fixture["routes"]}
    old_total = sum(row["wall_s"] for row in old_rows)
    new_total = sum(row["wall_s"] for row in new_rows)
    old_cloud, new_cloud = old["cloud"], new.get("cloud", {})
    cloud_hash = fixture["ply_sha256"]
    if old_cloud.get("sha256") != cloud_hash or new_cloud.get("sha256") != cloud_hash:
        raise ValueError("La huella de la nube debe coincidir con la referencia")
    report = Report(output)

    report.page("Tiempo", "Rendimiento del motor",
                "Diez rutas MAN10T sobre rimls.ply, de 10,00 s simulados cada una. "
                "Se emparejaron las entradas congeladas de la línea base de Python con la medición C++.")
    report.card(0, unit(old_total, "s"), "Python · suma del tiempo real")
    report.card(1, unit(new_total, "s"), "C++ · suma del tiempo real")
    report.card(2, fmt(old_total / new_total, 1) + "×", "Cociente observado Python / C++")
    report.legend_mark(LEFT + 35, 262, BLUE, "Python · Windows")
    report.legend_mark(LEFT + 177, 262, TEAL, "C++ · WSL2")
    horizontal_chart(report, pairs)
    rows = []
    for name, records in [("Python", old_rows), ("C++", new_rows)]:
        times = [row["wall_s"] for row in records]
        rows.append([name, fmt(min(times), 3), fmt(stats.median(times), 3),
                     fmt(stats.mean(times), 3), fmt(max(times), 3), f"{num_completed(records)} / 10"])
    end = report.table(641, ["Motor", "Mín. (s)", "Mediana (s)", "Media (s)", "Máx. (s)", "Completas"],
                       rows, [78, 77, 93, 83, 84, CONTENT - 415], range(1, 6))
    report.box(end + 14, "Comparación observacional",
               "El cronómetro empieza con el mapa cargado. El resultado refleja dos entornos de "
               "ejecución, bibliotecas distintas y una medición por ruta. El cociente cuantifica el "
               "cambio de extremo a extremo de esta prueba.", AMBER)
    report.end()

    report.page("Escenario", "Trayectorias y paridad observada",
                "Los recorridos comparten el mapa y las poses de partida. Las líneas muestran el "
                "movimiento calculado y permiten comprobar cuánto se separan las dos soluciones.")
    report.legend_mark(LEFT, 156, NAVY, "Python", True)
    report.legend_mark(LEFT + 101, 156, NAVY, "C++")
    report.legend_mark(LEFT + 186, 156, colors.HexColor("#8495A5"), "Guía de 8 m", True)
    report.para(146, "Punto en el extremo C++", 8.7, MUTED, LEFT + 342, 165)
    route_chart(report, pairs, guides)
    rows = []
    for before, after in pairs:
        rmse, final = path_differences(before, after)
        max_error = after.get("max_xy_error_m")
        rows.append([after["id"], fmt(before.get("path_m")), fmt(after.get("path_m")),
                     fmt(None if rmse is None else 1000 * rmse, 3),
                     fmt(None if final is None else 1000 * final, 3),
                     fmt(None if max_error is None else 1000 * max_error, 3)])
    end = report.table(489, ["Ruta", "Long. Py (m)", "Long. C++ (m)", "RMSE XY (mm)",
                             "Extremo (mm)", "Máx. XY (mm)"], rows,
                       [48, 80, 86, 99, 96, CONTENT - 409], range(1, 6), padding=3.4)
    maximum = max(r["max_xy_error_m"] for r in new_rows)
    same_decisions = sum(r["same_decisions"] for r in new_rows)
    report.para(end + 12,
                f"Las líneas se superponen a escala del mapa. Máxima separación entre cuadros "
                f"correspondientes: {unit(1000 * maximum, 'mm', 3)}. En {same_decisions}/10 rutas "
                f"coincidieron estado, avisos y número de muestras. Extremo mide la separación XY "
                "entre posiciones finales. El RMSE se calcula sobre los cuadros emparejados.", 9.2)
    report.end()

    report.page("Telemetría", "Uso de CPU y potencia de GPU",
                "La dinámica se calculó en CPU. La potencia eléctrica disponible procede del sensor "
                "de la GPU NVIDIA del equipo anfitrión. La CPU queda expresada en tiempo y utilización.")
    rows = []
    for before, after in pairs:
        rows.append([after["id"], fmt(before.get("cpu_s"), 3), fmt(after.get("cpu_s"), 3),
                     percent(before.get("cpu_pct")), percent(after.get("cpu_pct")),
                     fmt(before.get("gpu_mean_w"), 2), fmt(after.get("gpu_mean_w"), 2),
                     f'{before.get("gpu_n", 0)} / {after.get("gpu_n", 0)}'])
    end = report.table(159, ["Ruta", "CPU Py\n(s)", "CPU C++\n(s)", "CPU Py\n(%)",
                             "CPU C++\n(%)", "GPU Py\n(W)", "GPU C++\n(W)", "Muestras GPU\nPy / C++"], rows,
                       [44, 62, 63, 60, 63, 65, 65, CONTENT - 422], range(1, 8), padding=6)
    report.section(end + 36, "Lecturas agregadas")
    old_n, new_n = gpu_count(old_rows), gpu_count(new_rows)
    rows = []
    for label, run, records, n in [("Python", old, old_rows, old_n), ("C++", new, new_rows, new_n)]:
        energy = [r["gpu_energy_j"] for r in records if r.get("gpu_energy_j") is not None]
        valid_peak = [r["gpu_peak_w"] for r in records if r.get("gpu_peak_w") is not None]
        rows.append([label, f"{n} / 10", fmt(run.get("idle_gpu_mean_w")),
                     fmt(sample_mean(records, "gpu_mean_w")), fmt(max(valid_peak, default=None)),
                     fmt(sum(energy), 1) if len(energy) == 10 else "n/d"])
    end = report.table(end + 47, ["Motor", "Rutas con W", "Reposo (W)", "Media (W)",
                                  "Pico (W)", "Energía (J)"], rows,
                       [72, 91, 87, 85, 80, CONTENT - 415], range(1, 6))
    old_logical = old["environment"].get("logical_cpus")
    new_logical = new["environment"].get("logical_cpus")
    old_readings = sum(r.get("gpu_n", 0) for r in old_rows)
    new_readings = sum(r.get("gpu_n", 0) for r in new_rows)
    text = (f"CPU % = 100 × tiempo del proceso / (tiempo real × CPU lógicas). "
            f"Denominador de {old_logical} en Python y {new_logical} en C++. "
            f"GPU a intervalos solicitados de 200 ms: {old_readings} lecturas en rutas Python y "
            f"{new_readings} en C++. La media resume las medias por ruta. La energía estimada "
            "integra las lecturas y contiene el consumo de fondo de la GPU.")
    bottom = report.box(end + 16, "Cómo leer los valores", text)
    report.para(bottom + 13,
                "La potencia eléctrica de CPU quedó sin medir. El reposo de GPU difirió entre sesiones "
                "y las ventanas de C++ reúnen de 3 a 5 muestras. Estas condiciones limitan la comparación "
                "del consumo de GPU entre motores.", 9.4)
    report.end()

    functions, intervals, call_counts = read_gprof(gprof_paths)
    native_positive = [(symbol, seconds) for symbol, seconds in functions if seconds > 0]
    report.page("Perfilado", "Dónde se concentra el cálculo",
                "Pasadas adicionales con instrumentación. Se ordenan funciones por tiempo propio "
                "dentro de cada perfil y se identifica su papel en el motor.")
    report.section(161, "C++ · gprof")
    native_rows = [(function_name(symbol), fmt(seconds, 3), describe(function_name(symbol), NATIVE_NOTES))
                   for symbol, seconds in native_positive[:7]]
    if not native_rows:
        native_rows = [("Sin muestras positivas", "n/d", "El perfil bruto no atribuyó tiempo propio.")]
    end = report.table(172, ["Función o símbolo", "Propio (s)", "Trabajo observado"], native_rows,
                       [216, 78, CONTENT - 294], [1], size=8.6, padding=4.1)
    period = " / ".join(unit(value, "s", 3) for value in sorted(intervals)) if intervals else "n/d"
    native_total = sum(seconds for _, seconds in native_positive)
    detail = (f"gprof muestreó cada {period} y atribuyó {unit(native_total, 's', 3)}. "
              if intervals else f"gprof atribuyó {unit(native_total, 's', 3)}. ")
    if native_positive:
        leader, self_s = native_positive[0]
        detail += (f"{function_name(leader)} concentró el {percent(100 * self_s / native_total)} "
                   f"del tiempo propio registrado")
        detail += f" en {call_counts[leader]} llamadas. " if call_counts[leader] else ". "
    detail += "Las funciones expandidas en línea pueden figurar bajo su llamante."
    end += 12 + report.para(end + 10, detail, 9.3)
    report.section(end + 26, "Python · cProfile")
    py_rows = [(function_name(row["function"]), fmt(row["self_s"]), fmt(row["cum_s"]),
                describe(row["function"], PYTHON_NOTES)) for row in old["functions"][:5]]
    end = report.table(end + 37, ["Función", "Propio (s)", "Acum. (s)", "Trabajo observado"],
                       py_rows, [160, 73, 78, CONTENT - 311], [1, 2], size=8.6, padding=4.1)
    report.box(end + 16, "Dos instrumentos de medida",
               f"Las pasadas perfiladas sumaron {unit(profile_total(new), 's', 3)} en C++ "
               f"y {unit(profile_total(old), 's')} en Python. gprof muestrea CPU nativa y "
               "cProfile instrumenta llamadas Python e incluye trabajo delegado a bibliotecas. "
               "Cada reparto de costes se interpreta dentro de su propio perfil.", AMBER)
    report.end()

    report.page("Reproducción", "Protocolo y trazabilidad",
                "La línea base y la nueva ejecución conservan las entradas, los tiempos por ruta y "
                "los perfiles brutos para repetir la comparación.")
    old_env, new_env = old["environment"], new["environment"]
    rows = [
        ("Escenario", f"rimls.ply · {fmt(old_cloud['count'], 0)} puntos · MAN_10t · contactos completos"),
        ("Control", "R01 a R10 · poses asentadas · 0,01 s por paso · 10,00 s · CSV desactivado"),
        ("Plataforma Python", f"Windows nativo · Python {old_env.get('python', 'n/d').split()[0]} · "
         f"PyTorch {old_env.get('torch', 'n/d')} · {old_env.get('torch_threads', 'n/d')} hilos"),
        ("Plataforma C++", f"{new_env.get('platform', 'Linux / WSL2')} · "
         f"{new_env.get('compiler', 'compilador n/d')} · {new_env.get('build', 'Release')}"),
        ("Procesador", f"{old_env.get('cpu', 'n/d')} · {old_logical} lógicos en Python / {new_logical} en C++"),
        ("GPU", str(new_env.get("gpu") or old_env.get("gpu", "n/d"))),
        ("Plan de energía", str(new_env.get("power_plan") or "Equilibrado en la línea base")),
    ]
    end = report.table(150, ["Parámetro", "Configuración"], rows, [133, CONTENT - 133],
                       padding=7, size=9.3)
    report.section(end + 31, "Qué incluye el cronómetro")
    h = report.para(end + 42,
                    "Python mide la llamada al motor con la nube cargada. C++ mide el cálculo "
                    "nativo y la construcción de resultados de reproducción en memoria. La carga "
                    "del PLY, la serialización final, el navegador y el transporte quedan fuera.", 10)
    y = end + 42 + h + 24
    report.section(y, "Cómo repetir")
    h = report.para(y + 11,
                    "Verificar la huella del PLY y mantener las diez entradas en orden. Cargar el "
                    "mapa, calentar el motor durante 1 s simulado y medir 6 s de reposo de GPU. Hacer "
                    "la pasada cronometrada con pausas de 2 s antes de cada ruta. Repetir con "
                    "instrumentación según el README del paquete de medición.", 10)
    y += 11 + h + 24
    report.section(y, "Datos de referencia")
    h = report.para(y + 11,
                    "Python: informe original y metrics.json de output/man10t-work. "
                    "C++: metrics.json, gpu.csv y gprof.txt de output/man10t-profiling-cpp. "
                    "Los campos de CPU describen el proceso cronometrado.", 10)
    y += 11 + h + 18
    report.text(LEFT, y, "SHA-256 de rimls.ply", 9.2, TEAL, BOLD)
    report.text(LEFT, y + 17, cloud_hash, 9, NAVY, MONO)
    report.end()
    report.doc.save()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, default=BASE)
    parser.add_argument("--data", type=Path, default=DATA)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--gprof", type=Path, action="append")
    args = parser.parse_args()
    paths = args.gprof or [args.data / "gprof.txt"]
    make_report(args.baseline, args.data, args.out, paths)


if __name__ == "__main__":
    main()
