from flask import Flask
import csv

app = Flask(__name__)
CSV_FILE = "reading.csv"

@app.route("/")
def home():

    with open("reading.csv", "r") as file:
        rows = list(csv.reader(file))

    header = rows[0]
    data = rows[1:]
    
    latest = data[-1]
    
    recent = data[-10:]
    recent.reverse()

    timestamp = latest[0]
    soil = latest[1]

    log_rows = ""

    for row in recent:
        log_rows += f"""
        <tr>
            <td>{row[0]}</td>
            <td>{row[1]}</td>
        </tr>
        """

    return f"""
    <html>
    <head>
        <meta http-equiv="refresh" content="1">
        <title>NASA Soil Monitor</title>
    </head>

    <body>
        <h1>NASA Soil Monitor</h1>

        <h2>Current Soil Reading</h2>
        <p><b>Soil:</b> {soil}</p>
        <p><b>Time:</b> {timestamp}</p>

        <h2>Previous Logs</h2>

        <table border="1" cellpadding="8">
            <tr>
                <th>Time</th>
                <th>Soil</th>
            </tr>
            {log_rows}
        </table>
    </body>
    </html>
    """

if __name__ == "__main__":
    app.run(debug=True)