from datetime import datetime
from pathlib import Path
import csv
import json
import re

from flask import Flask, render_template_string
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


app = Flask(__name__)
CSV_FILE = Path(__file__).with_name("reading.csv")
MAX_CHART_POINTS = 120
ROLLING_WINDOW = 10
ONLINE_AFTER_SECONDS = 30


NUMBER = r"[-+]?\d+(?:\.\d+)?"


def number_after_label(payload, labels):
    """Return the first number following one of the supplied labels."""
    for label in labels:
        match = re.search(rf"{label}\s*[:=]\s*({NUMBER})", payload, re.IGNORECASE)
        if match:
            return float(match.group(1))
    return None


def parse_payload(payload):
    """Parse both the rover's current JSON and its older text messages."""
    payload = payload.strip()

    try:
        message = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        message = None

    if isinstance(message, dict):
        return {
            "sample_id": message.get("sample_id"),
            "moisture": message.get("moisture_percent"),
            "temperature": message.get("temperature_c"),
            "light": message.get("light_raw"),
        }

    return {
        "sample_id": None,
        "moisture": number_after_label(
            payload, [r"soil\s+moisture\s+level", r"moisture"]
        ),
        "temperature": number_after_label(payload, [r"temperature", r"temp"]),
        "light": number_after_label(payload, [r"light(?:_raw)?", r"photoresistor"]),
    }


def as_float(value):
    try:
        result = float(value)
        return result if np.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def load_readings():
    readings = []
    if not CSV_FILE.exists():
        return readings

    try:
        with CSV_FILE.open(newline="", encoding="utf-8-sig") as file:
            rows = csv.reader(file)
            next(rows, None)  # timestamp,soil

            for row in rows:
                if len(row) < 2:
                    continue

                try:
                    timestamp = datetime.strptime(row[0].strip(), "%Y-%m-%d %H:%M:%S")
                except ValueError:
                    continue

                values = parse_payload(row[1])
                moisture = as_float(values["moisture"])
                temperature = as_float(values["temperature"])
                light = as_float(values["light"])

                # Ignore messages such as "Hello from Arduino" that contain no sample.
                if moisture is None and temperature is None and light is None:
                    continue

                readings.append(
                    {
                        "timestamp": timestamp,
                        "sample_id": values["sample_id"],
                        "moisture": moisture,
                        "temperature": temperature,
                        "light": light,
                    }
                )
    except (OSError, csv.Error):
        return []

    return readings


def rolling_average(values, window=ROLLING_WINDOW):
    """Use NumPy to smooth a numeric series while preserving its length."""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return []

    result = np.empty(array.size, dtype=float)
    cumulative = np.cumsum(array)
    for index in range(array.size):
        start = max(0, index - window + 1)
        total = cumulative[index] - (cumulative[start - 1] if start else 0)
        result[index] = total / (index - start + 1)
    return result.tolist()


def build_chart(readings):
    chart_data = readings[-MAX_CHART_POINTS:]
    times = [reading["timestamp"] for reading in chart_data]
    moisture = [reading["moisture"] for reading in chart_data]
    temperature = [reading["temperature"] for reading in chart_data]
    light = [reading["light"] for reading in chart_data]

    valid_moisture = [(time, value) for time, value in zip(times, moisture) if value is not None]
    moisture_times = [item[0] for item in valid_moisture]
    moisture_values = [item[1] for item in valid_moisture]
    smoothed = rolling_average(moisture_values)

    figure = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.09,
        row_heights=[0.46, 0.27, 0.27],
        subplot_titles=("Soil moisture", "Temperature", "Ambient light"),
    )

    figure.add_trace(
        go.Scatter(
            x=moisture_times,
            y=moisture_values,
            name="Moisture",
            mode="lines+markers",
            line={"color": "#52d6a5", "width": 2},
            marker={"size": 5},
            hovertemplate="%{y:.1f}%<br>%{x|%b %d, %I:%M:%S %p}<extra></extra>",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=moisture_times,
            y=smoothed,
            name=f"{ROLLING_WINDOW}-sample average",
            mode="lines",
            line={"color": "#d8f3e8", "width": 3, "dash": "dot"},
            hovertemplate="Average: %{y:.1f}%<extra></extra>",
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=times,
            y=temperature,
            name="Temperature",
            mode="lines+markers",
            line={"color": "#ffb86b", "width": 2},
            marker={"size": 5},
            hovertemplate="%{y:.1f} °C<br>%{x|%b %d, %I:%M:%S %p}<extra></extra>",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=times,
            y=light,
            name="Light",
            mode="lines+markers",
            line={"color": "#ffd75e", "width": 2},
            marker={"size": 5},
            hovertemplate="Raw: %{y:.0f}<br>%{x|%b %d, %I:%M:%S %p}<extra></extra>",
        ),
        row=3,
        col=1,
    )

    figure.update_yaxes(title_text="%", range=[0, 105], row=1, col=1)
    figure.update_yaxes(title_text="°C", row=2, col=1)
    figure.update_yaxes(title_text="Raw", row=3, col=1)
    figure.update_xaxes(title_text="Sample time", row=3, col=1)
    figure.update_layout(
        height=700,
        margin={"l": 55, "r": 25, "t": 65, "b": 45},
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(7,20,17,0.55)",
        font={"color": "#d9ebe4", "family": "Inter, system-ui, sans-serif"},
        legend={"orientation": "h", "y": 1.08, "x": 0},
        hovermode="x unified",
        uirevision="terra-dashboard",
    )
    figure.update_xaxes(gridcolor="rgba(148,180,168,0.12)")
    figure.update_yaxes(gridcolor="rgba(148,180,168,0.12)", zeroline=False)

    return figure.to_html(
        full_html=False,
        include_plotlyjs="cdn",
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )


def display_value(value, decimals=1, suffix=""):
    return "—" if value is None else f"{value:.{decimals}f}{suffix}"


@app.route("/")
def home():
    readings = load_readings()

    if not readings:
        return render_template_string(
            DASHBOARD_TEMPLATE,
            has_data=False,
            chart_html="",
            latest=None,
            status="Waiting for data",
            status_class="waiting",
            moisture_state="No reading",
            moisture="—",
            temperature="—",
            light="—",
            rolling="—",
            sample_count=0,
            recent=[],
            updated="Not available",
        )

    latest = readings[-1]
    age_seconds = max(0, (datetime.now() - latest["timestamp"]).total_seconds())
    is_online = age_seconds <= ONLINE_AFTER_SECONDS

    moisture_values = [r["moisture"] for r in readings if r["moisture"] is not None]
    recent_values = moisture_values[-ROLLING_WINDOW:]
    average = float(np.mean(recent_values)) if recent_values else None

    latest_moisture = latest["moisture"]
    if latest_moisture is None:
        moisture_state = "No reading"
    elif latest_moisture < 30:
        moisture_state = "Low moisture"
    elif latest_moisture <= 70:
        moisture_state = "Moderate moisture"
    else:
        moisture_state = "High moisture"

    recent = []
    for reading in reversed(readings[-10:]):
        recent.append(
            {
                "time": reading["timestamp"].strftime("%b %d, %I:%M:%S %p"),
                "sample_id": reading["sample_id"] if reading["sample_id"] is not None else "—",
                "moisture": display_value(reading["moisture"], 1, "%"),
                "temperature": display_value(reading["temperature"], 1, " °C"),
                "light": display_value(reading["light"], 0),
            }
        )

    return render_template_string(
        DASHBOARD_TEMPLATE,
        has_data=True,
        chart_html=build_chart(readings),
        latest=latest,
        status="Receiving data" if is_online else "Rover offline",
        status_class="online" if is_online else "offline",
        moisture_state=moisture_state,
        moisture=display_value(latest["moisture"], 1, "%"),
        temperature=display_value(latest["temperature"], 1, " °C"),
        light=display_value(latest["light"], 0),
        rolling=display_value(average, 1, "%"),
        sample_count=len(readings),
        recent=recent,
        updated=latest["timestamp"].strftime("%B %d, %Y at %I:%M:%S %p"),
    )


DASHBOARD_TEMPLATE = r"""
<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="refresh" content="5">
    <title>TERRA Mission Control</title>
    <style>
        :root {
            color-scheme: dark;
            --bg: #06100d;
            --panel: rgba(16, 35, 29, 0.82);
            --panel-strong: #10231d;
            --line: rgba(169, 211, 195, 0.14);
            --text: #effaf6;
            --muted: #98b3a9;
            --green: #52d6a5;
            --amber: #ffbf69;
            --red: #ff7b7b;
        }

        * { box-sizing: border-box; }
        body {
            margin: 0;
            min-height: 100vh;
            color: var(--text);
            background:
                radial-gradient(circle at 85% 0%, rgba(45, 125, 94, 0.28), transparent 34%),
                radial-gradient(circle at 0% 70%, rgba(36, 84, 68, 0.18), transparent 28%),
                var(--bg);
            font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        }

        .shell { width: min(1440px, calc(100% - 32px)); margin: 0 auto; padding: 28px 0 48px; }
        .topbar { display: flex; align-items: center; justify-content: space-between; gap: 24px; margin-bottom: 26px; }
        .brand { display: flex; align-items: center; gap: 14px; }
        .mark {
            display: grid; place-items: center; width: 48px; height: 48px; border-radius: 15px;
            background: linear-gradient(145deg, #67e9b8, #20785b); color: #04100c; font-weight: 900;
            box-shadow: 0 12px 30px rgba(36, 178, 126, 0.22);
        }
        .eyebrow { color: var(--green); font-size: 0.72rem; font-weight: 800; letter-spacing: 0.16em; text-transform: uppercase; }
        h1 { margin: 3px 0 0; font-size: clamp(1.45rem, 3vw, 2rem); letter-spacing: -0.03em; }
        .status { display: inline-flex; align-items: center; gap: 9px; padding: 10px 14px; border: 1px solid var(--line); border-radius: 999px; background: rgba(9, 24, 19, 0.72); color: var(--muted); font-size: 0.86rem; }
        .status::before { content: ""; width: 9px; height: 9px; border-radius: 50%; background: var(--amber); box-shadow: 0 0 0 4px rgba(255,191,105,.12); }
        .status.online::before { background: var(--green); box-shadow: 0 0 0 4px rgba(82,214,165,.12), 0 0 18px rgba(82,214,165,.7); }
        .status.offline::before { background: var(--red); box-shadow: 0 0 0 4px rgba(255,123,123,.12); }

        .hero { display: grid; grid-template-columns: 1.4fr 1fr; gap: 16px; margin-bottom: 16px; }
        .panel { border: 1px solid var(--line); background: var(--panel); border-radius: 22px; box-shadow: 0 20px 55px rgba(0,0,0,.18); backdrop-filter: blur(14px); }
        .primary { padding: 28px; min-height: 220px; display: flex; flex-direction: column; justify-content: space-between; overflow: hidden; position: relative; }
        .primary::after { content: ""; position: absolute; width: 250px; height: 250px; border-radius: 50%; right: -90px; bottom: -150px; border: 34px solid rgba(82,214,165,.06); }
        .label { color: var(--muted); font-size: .77rem; font-weight: 750; letter-spacing: .12em; text-transform: uppercase; }
        .big-value { margin: 8px 0 0; font-size: clamp(3.5rem, 9vw, 6.5rem); font-weight: 800; line-height: .95; letter-spacing: -.07em; }
        .descriptor { color: var(--green); font-size: .95rem; font-weight: 700; }
        .updated { color: var(--muted); font-size: .82rem; }

        .stats { display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px; }
        .stat { padding: 21px; min-height: 102px; display: flex; flex-direction: column; justify-content: space-between; }
        .stat-value { margin-top: 9px; font-size: 1.65rem; font-weight: 760; letter-spacing: -.04em; }

        .chart { padding: 12px 12px 2px; margin-bottom: 16px; overflow: hidden; }
        .section-head { display: flex; justify-content: space-between; align-items: end; gap: 20px; padding: 18px 20px 0; }
        .section-head h2 { margin: 4px 0 0; font-size: 1.12rem; }
        .section-note { color: var(--muted); font-size: .78rem; }

        .logs { padding: 18px 20px 10px; overflow: hidden; }
        .table-wrap { overflow-x: auto; margin: 12px -20px 0; }
        table { width: 100%; border-collapse: collapse; min-width: 710px; }
        th, td { padding: 14px 20px; text-align: left; border-top: 1px solid var(--line); }
        th { color: var(--muted); font-size: .72rem; letter-spacing: .1em; text-transform: uppercase; }
        td { font-size: .87rem; }
        tbody tr:hover { background: rgba(82,214,165,.035); }
        .empty { text-align: center; padding: 70px 20px; color: var(--muted); }

        @media (max-width: 850px) {
            .shell { width: min(100% - 20px, 720px); padding-top: 18px; }
            .topbar, .hero { align-items: flex-start; }
            .hero { grid-template-columns: 1fr; }
            .topbar { flex-direction: column; gap: 14px; }
        }
        @media (max-width: 520px) {
            .stats { grid-template-columns: 1fr; }
            .primary { min-height: 190px; }
            .section-note { display: none; }
        }
    </style>
</head>
<body>
    <main class="shell">
        <header class="topbar">
            <div class="brand">
                <div class="mark">T</div>
                <div>
                    <div class="eyebrow">NASA Space Grant</div>
                    <h1>TERRA Mission Control</h1>
                </div>
            </div>
            <div class="status {{ status_class }}">{{ status }}</div>
        </header>

        {% if has_data %}
        <section class="hero">
            <article class="panel primary">
                <div>
                    <div class="label">Current soil moisture</div>
                    <div class="big-value">{{ moisture }}</div>
                </div>
                <div>
                    <div class="descriptor">{{ moisture_state }}</div>
                    <div class="updated">Last sample: {{ updated }}</div>
                </div>
            </article>

            <div class="stats">
                <article class="panel stat"><div class="label">Temperature</div><div class="stat-value">{{ temperature }}</div></article>
                <article class="panel stat"><div class="label">Ambient light</div><div class="stat-value">{{ light }}</div></article>
                <article class="panel stat"><div class="label">10-sample average</div><div class="stat-value">{{ rolling }}</div></article>
                <article class="panel stat"><div class="label">Valid samples</div><div class="stat-value">{{ "{:,}".format(sample_count) }}</div></article>
            </div>
        </section>

        <section class="panel chart">
            <div class="section-head">
                <div><div class="label">Telemetry</div><h2>Recent sensor history</h2></div>
                <div class="section-note">Latest {{ 120 if sample_count > 120 else sample_count }} samples · refreshes every 5 seconds</div>
            </div>
            {{ chart_html | safe }}
        </section>

        <section class="panel logs">
            <div class="section-head" style="padding: 0">
                <div><div class="label">Field log</div><h2>Latest readings</h2></div>
            </div>
            <div class="table-wrap">
                <table>
                    <thead><tr><th>Time</th><th>Sample</th><th>Moisture</th><th>Temperature</th><th>Light</th></tr></thead>
                    <tbody>
                        {% for row in recent %}
                        <tr><td>{{ row.time }}</td><td>{{ row.sample_id }}</td><td>{{ row.moisture }}</td><td>{{ row.temperature }}</td><td>{{ row.light }}</td></tr>
                        {% endfor %}
                    </tbody>
                </table>
            </div>
        </section>
        {% else %}
        <section class="panel empty">
            <h2>Waiting for the first sensor reading</h2>
            <p>Start <code>udp_listener.py</code> and this dashboard will populate automatically.</p>
        </section>
        {% endif %}
    </main>
</body>
</html>
"""


if __name__ == "__main__":
    app.run(debug=True)