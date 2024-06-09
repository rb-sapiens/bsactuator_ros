# bsactuator_ros
RoboSapiensが開発するBambooshoot ActuatorとやりとりをするためのROSパッケージです。

## 事前準備
`bsactuator` ライブラリをインストールしている必要があります。

```
pip3 install git+https://github.com/rb-sapiens/bsactuator.git
```

また、 `/etc/udev/rules.d/` にudevファイルを置く必要があります。（USB接続を認識するため）
```
cd bsactuator_ros
mv bambooshoot_actuator.rules /etc/udev/rules.d/
```

## Launchファイルの実行
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