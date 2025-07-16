![ROS2-humble Industrial CI](https://github.com/Yutarop/sign_detection_img/actions/workflows/ros2_ci.yml/badge.svg)
# つくばチャレンジ2024 選択課題C    
つくばチャレンジ選択課題Cで使用される経路封鎖看板を検出するアルゴリズムのROS2パッケージです。転移学習済みのYOLOv8モデルを用いて、看板を検出します。  

[ポイントクラウドデータのみで看板の検出をしたい場合](https://github.com/Yutarop/sign_detection)

## 使用モデル
本ノードで使用しているYOLOv8モデル（`sign_detection.pt`）は、以下の条件で学習されたものです。
- 訓練データ数: 270枚
- テストデータ数: 117枚
- 検出対象: 経路封鎖看板（signboard）
- 事前学習済みモデル: YOLOv8n

以下のURLからデータセットをダウンロードできます。  
[Hugging Face](https://huggingface.co/datasets/Yutatt/tsukuba-closed-signboard), [Kaggle](https://www.kaggle.com/datasets/qdgvhz9qyf9v/tsukuba-signboard-dataset/data)

## デモ
https://github.com/user-attachments/assets/ea85cbb4-2bb2-42b3-92bf-44f413820fc2

## 使用方法
#### リポジトリをクローンする
```bash
cd ~/ros2_ws/src
git clone git@github.com:Yutarop/sign_detection_img.git
```
#### 依存パッケージのインストール
```bash
pip install opencv-python ultralytics
```

#### ワークスペースのビルド
```bash
cd ~/ros2_ws
colcon build --packages-select sign_detection_img
source ~/ros2_ws/install/setup.bash
```
#### ノードの実行
```bash
ros2 run sign_detection_img sign_detection_img
```
