import argparse
import picamera
import shutil
import tempfile
import time
import google.auth
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

parser = argparse.ArgumentParser(description='Record a video from the Raspberry Pi camera')
parser.add_argument('--preview', action='store_true', help='Show a preview of the camera')
parser.add_argument('duration', type=int, help='Duration of the video in seconds')

args = parser.parse_args()

# Get credentials for the Google Drive API
creds, project_id = google.auth.default()

# Create a temporary file for the video
timestamp = time.strftime("%Y%m%d-%H%M%S")
local_file = f"{timestamp}.h264"
temp_file = tempfile.NamedTemporaryFile(suffix='.h264')

with picamera.PiCamera(sensor_mode=4) as camera:
    camera.resolution = (1296, 972)
    camera.framerate = 42
    if args.preview:
        camera.start_preview()
    camera.start_recording(temp_file.name)
    camera.wait_recording(args.duration)
    camera.stop_recording()

# Upload the video to Google Drive
try:
    service = build('drive', 'v3', credentials=creds)
    file_metadata = {'name': local_file}
    media = MediaFileUpload(temp_file.name,
                            mimetype='video/h264')
    file = service.files().create(body=file_metadata, media_body=media,
                                  fields='id').execute()
    print(F'File ID: {file.get("id")}')

except HttpError as error:
    print(F'An error occurred while uploading the video: {error}')
    print(F'Saving the video locally as {local_file}')
    shutil.copy(temp_file.name, local_file)
    file = None

# Clean up the temporary file
temp_file.close()
shutil.rmtree(temp_file.name)

