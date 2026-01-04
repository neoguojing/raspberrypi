import threading
import time
import json

import zenoh

# =============================
# Zenoh Subscriber (B)
# =============================
def zenoh_subscriber():
    config = zenoh.Config()
    config.insert_json5(
        "connect/endpoints",
        '["tcp/127.0.0.1:7447"]'
    )

    session = zenoh.open(config)
    print("Zenoh Subscriber connected")
    print("Session id:", session.id)
    def callback(sample):
        data = bytes(sample.payload).decode("utf-8")
        print(f"Zenoh Subscriber received: {sample.key_expr} -> {data}")
    
    sub = session.declare_subscriber("rt/test_ros2_to_zenoh", callback)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        session.close()

# =============================
# Zenoh Publisher (C)
# =============================
def zenoh_publisher():
    config = zenoh.Config()
    config.insert_json5(
        "connect/endpoints",
        '["tcp/127.0.0.1:7447"]'
    )

    session = zenoh.open(config)
    pub = session.declare_publisher("rt/test_zenoh_to_ros2")

    count = 0
    try:
        while True:
            msg = f"Hello from Zenoh {count}"
            pub.put(payload=msg.encode("utf-8"))
            print(f"Zenoh Published: {msg}")
            count += 1
            time.sleep(1)
    except KeyboardInterrupt:
        session.close()

# =============================
# Main
# =============================
def main():
    # Zenoh threads
    t_sub = threading.Thread(target=zenoh_subscriber, daemon=True)
    t_pub = threading.Thread(target=zenoh_publisher, daemon=True)
    
    t_sub.start()
    t_pub.start()

    try:
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n👋 正在关闭 Zenoh 节点...")

if __name__ == '__main__':
    main()
