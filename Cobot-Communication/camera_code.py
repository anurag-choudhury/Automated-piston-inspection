from pypylon import pylon
import cv2
import os

def click_image(flag, counter):

    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    # Demonstrate some feature access
    new_width = camera.Width.GetValue() - camera.Width.GetInc()
    if new_width >= camera.Width.GetMin():
        camera.Width.SetValue(new_width)

    numberOfImagesToGrab = 1
    camera.StartGrabbingMax(numberOfImagesToGrab)

    img = None
    while camera.IsGrabbing():
        grabResult = camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            img = grabResult.Array

        grabResult.Release()
    
    camera.Close()

    # Directories for saving images
    directory = r'/fsm_piston/src/Cobot-Communication/images_camera/'
    directory_crown = r'/fsm_piston/src/Cobot-Communication/images_camera/crown_img'
    directory_ring = r'/fsm_piston/src/Cobot-Communication/images_camera/ring_zone_img'

    # Choose directory and filename based on the flag
    if flag == 0:
        os.chdir(directory_crown)
        image_filename = f'sample_{counter}.jpg'
        cv2.imwrite(image_filename, img)
        image_path = os.path.join(directory_crown, image_filename)
    else:
        os.chdir(directory_ring)
        image_filename = f'sample_{counter}_img_{flag}.jpg'
        cv2.imwrite(image_filename, img)
        image_path = os.path.join(directory_ring, image_filename)

    # Change back to the original directory
    os.chdir(directory)

    # Return the saved image path
    return image_path
