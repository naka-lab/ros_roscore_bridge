# ros_roscore_bridge

異なるroscoreで管理されているネットワーク間のメッセージをSubscribe, Publishするプログラム

## 準備
```
cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_roscore_bridge.git
```

## 使い方
- Publisher, Subscriber: [example.py](scripts/example.py)
- Service: [example_service.py](scripts/example_service.py)
- Action: [example_action.py](scripts/example_action.py)
- 任意のコードの実行: [example_execcode.py](scripts/example_execcode.py)

## 制限事項
- メッセージは，コールバック関数または`get_message`で取得できますが，一方で取得されたデータはキューから削除されるため，これらの同時利用はできません．
