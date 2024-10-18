from flask import Flask, request, jsonify, send_file
import torch
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO
import os, time, io
import base64

# Initialize the Flask app
app = Flask(__name__)

# Load the trained YOLO model
model = YOLO('./model/best.pt')  # Replace with your model path

def draw_bounding_boxes(img_np, predictions):
    """
    Draws bounding boxes and labels on the image.
    """
    # Convert numpy image to PIL Image for easy annotation
    img_pil = Image.fromarray(img_np)
    draw = ImageDraw.Draw(img_pil)

    # Loop over each detection and draw the bounding box
    for detection in predictions:
        x1, y1, x2, y2, confidence, class_id = detection.cpu().numpy()
        if int(class_id) == 1 and confidence >= 0.15:  # Filter based on confidence and class_id
            # Draw a rectangle (bounding box)
            draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
            # Annotate with defect label and confidence
            draw.text((x1, y1), f"Defect: {confidence:.2f}", fill="red")
    
    # Convert PIL Image back to numpy array
    img_with_boxes = np.array(img_pil)
    return img_with_boxes

@app.route('/predict', methods=['POST'])
def predict():
    try:
        # Get the image file from the POST request
        file = request.files['image']

        # Convert the image file to a PIL Image
        img = Image.open(file.stream).convert('RGB')  # Ensure image is in RGB format
        print("Image shape before resizing:", np.array(img).shape)

        # Resize the image to match model input size (YOLO typically works with 640x640)
        img = img.resize((640, 480))
        print("Image shape after resizing:", np.array(img).shape)

        # Convert the image to a numpy array
        img_np = np.array(img)
        # cv2.imshow("model image",img_np)
        # cv2.waitKey(2000) 
        # Run YOLO model on the image
        results = model(img_np)  # Ensure numpy array is passed to the model
        print("results,",results)
        # Parse the results to extract bounding boxes and class info
        predictions = results[0].boxes.data  # Tensor containing [x1, y1, x2, y2, confidence, class]
        print(predictions)  # Log predictions for debugging

        # Initialize response
        response = {"status": "non-defected"}  # Default to non-defected
        img_with_boxes = img_np

        # Check for detections
        if predictions.size(0) > 0:
            for detection in predictions:
                x1, y1, x2, y2, confidence, class_id = detection.cpu().numpy()
                print(f"Detected: Class ID {class_id}, Confidence: {confidence}, BBox: ({x1}, {y1}, {x2}, {y2})")
                if int(class_id) == 1 and confidence >= 0.15:  # Adjust threshold as necessary
                    response = {"status": "defected", "confidence": float(confidence)}
                    # Draw bounding boxes on the image if a defect is detected
                    img_with_boxes = draw_bounding_boxes(img_np, predictions)
                    break
        
        # At this point, img_with_boxes contains the image with or without bounding boxes
        cv_image = np.array(img_with_boxes)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        
        # Set the color for the status text
        if response['status'] == "defected":
            color = (0, 0, 255)  # Red color for defected
        else:
            color = (0, 255, 0)  # Green color for non-defected

        # Add the status text to the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_position = (10, 30)
        font_scale = 1
        thickness = 2
        cv2.putText(cv_image, response['status'], text_position, font, font_scale, color, thickness)

        # Convert the image with bounding boxes and text back to PIL and then to base64
        img_pil_with_boxes = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        buffered = io.BytesIO()
        img_pil_with_boxes.save(buffered, format="JPEG")
        img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')

        # Add the image to the response
        response["image"] = img_base64

        return jsonify(response)  # Return the classification result and image as JSON

    except Exception as e:
        return jsonify({"error": str(e)})

@app.route('/')
def home():
    return jsonify({
        'message': 'Welcome to the Flask API!',
        'status': 'success'
    })

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
