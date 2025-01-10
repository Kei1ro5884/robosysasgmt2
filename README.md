# robosysasgmt2
このパッケージは、OpenWeatherMapのAPIを使用して特定の都市の天気情報を取得し、配信するノードを含むROS2のパッケージです。

[![test](https://github.com/Kei1ro5884/robosysasgmt2/actions/workflows/test.yml/badge.svg)](https://github.com/Kei1ro5884/robosysasgmt2/actions/workflows/test.yml)

## テスト環境
- Ubuntu 22.04 LTS

## ノード

### weather_forecast
OpenWeatherMapのAPIを使用して、指定した都市の天気情報を取得し、ROS2トピックを通じて配信するノードです。

### 主な機能
- 指定した都市の現在の天気、気温などの情報を取得します。
- 取得した情報をROS2のトピック /weather_forecast で配信します。

### 配信するトピック
- トピック名: /weather_forecast
- 型: std_msgs/msg/String
- 役割: 現在の天気情報を文字列形式で送信します。

## 実行方法
以下のコマンドでノードを実行できます。
```
ros2 run robosysasgmt2 weather_forecast
```

## 実行結果
以下は、ノード実行時のログ例です。
```
[INFO] [1736516114.031917951] [weather_forecast]: WeatherForecastNode has been started.
```

## トピックの確認方法

### 実行方法
以下のコマンドで /weather_forecast の内容を確認できます。
```
ros2 topic echo /weather_forecast
```

### 実行結果
```
data: City: Tokyo, Weather: clear sky, Temp: 3.7°C
---
```

## APIの著作権
- このパッケージは [OpenWeatherMap API](https://openweathermap.org/api) を利用しています。
- APIキーの取得には、OpenWeatherMapのアカウント作成および無料プランへの登録が必要です。

## ライセンス
- このソフトウェアパッケージは, 3条項BSDライセンスの下, 再頒布および使用が許可されます.
- © 2025 Keiichiro Kobayashi
