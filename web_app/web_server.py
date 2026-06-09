from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from pathlib import Path
import json

app = FastAPI()

BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = BASE_DIR / "web_config.json"
RESULTS_DIR = BASE_DIR.parent / "vision_workspace" / "operation" / "results"

app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")
app.mount("/results", StaticFiles(directory=RESULTS_DIR), name="results")

templates = Jinja2Templates(directory=BASE_DIR / "templates")


@app.get("/", response_class=HTMLResponse)
def home(request: Request):
    return templates.TemplateResponse(
        request=request,
        name="index.html" 
    )


@app.get("/api/config")
def get_config():
    with open(CONFIG_PATH, "r") as f:
        return json.load(f)


@app.post("/api/config")
def save_config(config: dict):
    print("Saving config to:", CONFIG_PATH)
    print("New config:", config)

    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=4)

    return {
        "status": "saved",
        "saved_to": str(CONFIG_PATH),
        "config": config
    }

@app.get("/api/latest-image")
def latest_image():
    print("Looking for images in:", RESULTS_DIR)
    print("Folder exists:", RESULTS_DIR.exists())

    image_files = (
        list(RESULTS_DIR.glob("*.jpg")) +
        list(RESULTS_DIR.glob("*.jpeg")) +
        list(RESULTS_DIR.glob("*.png"))
    )

    print("Images found:", image_files)

    if not image_files:
        return {"image": None}

    newest = max(image_files, key=lambda p: p.stat().st_mtime)
    return {"image": f"/results/{newest.name}"}