from roboflow import Roboflow
rf = Roboflow(api_key="bu6wMUcHuTQ0DegmfkpB")
project = rf.workspace("sjsu-ilbp6").project("anomaly-dqrqf")
version = project.version(11)
dataset = version.download("yolov5")
                