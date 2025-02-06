# bsactuator_ros

RoboSapiens が開発する Bambooshoot Actuator とやりとりをするための ROS パッケージです。

## 事前準備

`bsactuator` ライブラリをインストールしている必要があります。
対応バージョン：0.2.1 以上

```
pip3 install git+https://github.com/rb-sapiens/bsactuator.git
```

また、 `/etc/udev/rules.d/` に udev ファイルを置く必要があります。（USB 接続を認識するため）

```
cd bsactuator_ros
mv bambooshoot_actuator.rules /etc/udev/rules.d/
```

## Launch ファイルの実行

```
roslaunch bsactuator_ros bsactuator.launch
```

## Subscribers

#### /set_length

伸縮長さを指定する

ex.

```
rostopic pub -1 /set_length std_msgs/Int16 300
```

## Publishers

#### /bsactuator/length

現在の長さ[mm]

#### /bsactuator/status

伸縮が終わった時に発行されるステータス
