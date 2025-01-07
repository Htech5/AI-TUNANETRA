import os
from PIL import Image
from ultralytics import YOLO
import yaml
import torch

# Fungsi untuk meresize gambar
def resize_images(input_folder, output_folder, size=(640, 640)):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in os.listdir(input_folder):
        if filename.endswith(('.png', '.jpg', '.jpeg')):  # Memeriksa ekstensi gambar
            img_path = os.path.join(input_folder, filename)
            img = Image.open(img_path)
            img = img.resize(size, Image.LANCZOS)  # Mengubah ukuran gambar dengan metode LANCZOS
            img.save(os.path.join(output_folder, filename))  # Menyimpan gambar yang telah diresize

# Membaca file data.yaml 
data_yaml_path = 'D:/PROGRAM/AI TUNATERA/data/train.yaml'
with open(data_yaml_path, 'r') as file:
    data = yaml.safe_load(file)

# Mendapatkan path folder dari data.yaml
train_folder = data['train']
val_folder = data['val']
test_folder = data['test']

# Pastikan folder path adalah string (bukan list)
if isinstance(train_folder, list):
    train_folder = train_folder[0]  # Ambil elemen pertama jika itu adalah list
if isinstance(val_folder, list):
    val_folder = val_folder[0]  # Ambil elemen pertama jika itu adalah list
if isinstance(test_folder, list):
    test_folder = test_folder[0]  # Ambil elemen pertama jika itu adalah list

# Menentukan folder output untuk gambar yang diresize
train_resized_folder = os.path.join(os.path.dirname(train_folder), 'resized_train')
val_resized_folder = os.path.join(os.path.dirname(val_folder), 'resized_val')
test_resized_folder = os.path.join(os.path.dirname(test_folder), 'resized_test')

# Resize semua gambar di folder train, val, dan test
resize_images(train_folder, train_resized_folder)
resize_images(val_folder, val_resized_folder)
resize_images(test_folder, test_resized_folder)

# Melatih model YOLO dengan folder yang telah diresize
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = YOLO('yolov8n.pt').to(device)  # Pastikan model dipindahkan ke GPU

# Menampilkan informasi perangkat yang digunakan
print(f"Using device: {device}")

# Melatih model
model.train(data=data_yaml_path, epochs=100, imgsz=640, batch=8, patience=128, device=device)
