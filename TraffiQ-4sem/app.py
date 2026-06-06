from flask import Flask, request, redirect, render_template, jsonify, session
import sqlite3
import os
from simulation.sumo import get_lane_data, start_sumo, stop_sumo
import atexit

app = Flask(
    __name__,
    template_folder="fronted"
)

app.secret_key = "traffic_secret_key"

DB_PATH = "users.db"

def init_db():
    con = sqlite3.connect(DB_PATH)
    cur = con.cursor()
    cur.execute('''
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT NOT NULL,
            email TEXT NOT NULL,
            password TEXT NOT NULL
        )
    ''')
    con.commit()
    con.close()

init_db()

start_sumo()
atexit.register(stop_sumo)

@app.route('/api/traffic')
def traffic_data():
    data = get_lane_data()
    return jsonify(data)



@app.route('/')
def home():
    return render_template("index.html")


@app.route('/login')
def login_page():
    return render_template("login.html")

@app.route('/signup')
def signup_page():
    return render_template("signup.html")


@app.route('/signup', methods=['POST'])
def signup():
    username = request.form['username']
    email = request.form['email']
    password = request.form['password']

    con = sqlite3.connect(DB_PATH)
    cur = con.cursor()
    cur.execute("SELECT * FROM users WHERE username=? OR email=?", (username, email))
    existing = cur.fetchone()

    if existing:
        con.close()
        return render_template("signup.html", message="User already exists")

    cur.execute("INSERT INTO users (username, email, password) VALUES (?, ?, ?)",
                (username, email, password))
    con.commit()
    con.close()

    session['user'] = username
    return redirect("/dashboard")


@app.route('/login', methods=['POST'])
def login():
    username = request.form['username']
    password = request.form['password']

    con = sqlite3.connect(DB_PATH)
    cur = con.cursor()
    cur.execute("SELECT username, password FROM users WHERE username=?", (username,))
    user = cur.fetchone()
    con.close()

    if not user:
        return render_template("login.html", message="Invalid username or password")

    db_username, db_password = user

    if db_password == password:
        session['user'] = db_username
        return redirect("/dashboard")
    else:
        return render_template("login.html", message="Invalid username or password ")



@app.route('/dashboard')
def dashboard():
    if 'user' not in session:
        return redirect("/login")
    return render_template("dashboard.html", user=session['user'])



@app.route('/logout')
def logout():
    session.pop('user', None)
    return redirect("/login")


if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)