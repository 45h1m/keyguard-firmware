const express = require("express");
const multer = require("multer");
const fs = require("fs");
const path = require("path");

const app = express();
const PORT = 3000;

const response = {
    success: true,
    recognition_result: "identified", // or "unrecognized", "no_permission"
    timestamp: "2025-04-20T14:30:46Z",
    message: "Access granted for John Doe",
    actions: [
        {
            lock_id: "lock_002",
            action: "unlock",
            duration: 5, // seconds to keep unlocked
        },
    ],
    person: {
        id: "user_42",
        name: "John Doe",
        access_level: 2,
    },
};

// Configure Multer to store files in memory (for debugging)
const upload = multer({
    storage: multer.memoryStorage(),
    limits: { fileSize: 10 * 1024 * 1024 }, // 10MB max
    fileFilter: (req, file, cb) => {
        if (file.mimetype === "image/jpeg") {
            cb(null, true);
        } else {
            cb(new Error("Only JPEG images are allowed!"), false);
        }
    },
});

// Handle POST request from ESP32-CAM
app.post("/upload", upload.single("imageFile"), (req, res) => {
    if (!req.file) {
        return res.status(400).send("No file uploaded or invalid format.");
    }

    // Save the image to disk (for verification)
    const fileName = `esp32-cam-${Date.now()}.jpg`;
    fs.writeFileSync(`uploads/${fileName}`, req.file.buffer);

    console.log(`Image saved: uploads/${fileName}`);
    res.status(200).json(response);
});

// Error handling
app.use((err, req, res, next) => {
    if (err instanceof multer.MulterError) {
        res.status(400).send("File upload error: " + err.message);
    } else if (err) {
        res.status(500).send("Server error: " + err.message);
    }
    next();
});

// Create 'uploads' directory if it doesn't exist
if (!fs.existsSync("uploads")) {
    fs.mkdirSync("uploads");
}

// Start server
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
});
