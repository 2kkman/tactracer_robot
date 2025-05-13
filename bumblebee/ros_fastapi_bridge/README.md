# ROS-FastAPI Bridge

A lightweight bridge between **HTTP GET requests** and **ROS Noetic** topics using **FastAPI**.  
This allows external systems (e.g., IoT devices, servers) to control ROS nodes via simple web requests.

---

## 🚀 Features

- ✅ Publish string data to ROS topics via HTTP
- ✅ Auto-initializes ROS publisher per topic
- ✅ Scalable and modular code structure
- ✅ Lightweight, fast, and easy to extend

---

## 🧪 Example Usage

### HTTP Request

```http
GET http://<host>:6001/control?topicname=/INFO&datastr=HELLOWORLD

Expected Behavior
Publishes a std_msgs/String message "HELLOWORLD" to the /INFO topic.

📦 Project Structure
graphql
복사
편집
ros_fastapi_bridge/
├── main.py                # FastAPI entry point
├── ros_publisher.py       # ROS publisher logic
├── routers/
│   └── control.py         # HTTP router for /control
├── requirements.txt       # Python dependencies
└── README.md              # Project documentation
⚙️ Requirements
Ubuntu 20.04

ROS Noetic

Python 3.8+

rospy, fastapi, uvicorn

Install Python dependencies:

bash
복사
편집
pip install -r requirements.txt
🧭 How to Run
1. Start ROS core
bash
복사
편집
source /opt/ros/noetic/setup.bash
roscore
2. Run the FastAPI server
bash
복사
편집
uvicorn main:app --host 0.0.0.0 --port 6001
🧩 Extend This Project
You can easily extend the bridge to:

🔁 Support other ROS message types (e.g., Twist, Int32, Pose)

🛰 Call ROS services from HTTP

🔐 Add authentication (e.g., API keys, JWT)

📈 Integrate monitoring/logging tools

🛠 Add a web-based control panel

🔒 Reboot System via Python (root only)
If needed, here’s a one-liner to safely reboot the system:

python
복사
편집
import os
os.system("reboot")
📜 License
MIT License

👨‍💻 Author
Created by [Your Name]
Contributions and suggestions welcome!

yaml
복사
편집
