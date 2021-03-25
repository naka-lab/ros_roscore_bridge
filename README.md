# ros_roscore_bridge

異なるroscoreで管理されているネットワーク間のメッセージをSubscribe, Publishするプログラム

## 準備
```
cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_roscore_bridge.git
```

## 使い方
[example.py](scripts/example.py)を参照

## 制限事項
- プロセス間通信を利用している都合上，Publiser, Subscriberの細かな制御はできません．
- メッセージは，コールバック関数または`get_message`で取得できますが，一方で取得されたデータはキューから削除されるため，これらの同時利用はできません．
