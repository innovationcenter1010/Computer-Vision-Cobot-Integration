async function loadConfig() {
    const response = await fetch("/api/config");
    const config = await response.json();

    document.getElementById("robot_ip").value = config.robot_ip;
    document.getElementById("camera_ip").value = config.camera_ip;
    document.getElementById("save_folder").value = config.save_folder;
    document.getElementById("confidence_threshold").value = config.confidence_threshold;
    document.getElementById("enable_motion").checked = config.enable_motion;
}

async function saveConfig() {
    const config = {
        robot_ip: document.getElementById("robot_ip").value,
        camera_ip: document.getElementById("camera_ip").value,
        save_folder: document.getElementById("save_folder").value,
        confidence_threshold: parseFloat(document.getElementById("confidence_threshold").value),
        enable_motion: document.getElementById("enable_motion").checked
    };

    console.log("Saving config:", config);

    const response = await fetch("/api/config", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(config)
    });

    const result = await response.json();

    console.log("Server response:", result);

    document.getElementById("save_status").innerText = "Saved!";
}

async function loadLatestImage() {
    const response = await fetch("/api/latest-image");
    const result = await response.json();

    const img = document.getElementById("latest_image");

    if (result.image === null) {
        img.src = "";
        img.alt = "No image found";
    } else {
        img.src = result.image + "?t=" + new Date().getTime();
        img.alt = "Latest image";
    }
}

loadConfig();
loadLatestImage();