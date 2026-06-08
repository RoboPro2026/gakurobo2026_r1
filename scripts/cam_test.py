# PCのカメラで動作確認するやつ
import cv2


def main():
    seed = 1108
    # 1. カスタム辞書の生成 (マーカー数50, マーカーサイズ4x4, シード値12345)
    # シード値を固定すれば、何度実行しても同じカスタム辞書が生成されます
    custom_dict = cv2.aruco.extendDictionary(50, 4, randomSeed=seed)

    # 2. 検出用パラメータと検出器の設定
    # OpenCV 4.7.0以降の推奨される書き方
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(custom_dict, parameters)

    # 3. ウェブカメラの起動 (通常 0番)
    # cap = cv2.VideoCapture(3)
    cap = cv2.VideoCapture(0)

    # C1カメラの場合
    # cap = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)

    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"UYVY"))
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)
    # cap.set(cv2.CAP_PROP_FPS, 30)

    print("カメラを起動しました。'q' キーで終了します。")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 4. マーカーの検出
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.equalizeHist(gray)

        corners, ids, rejected = detector.detectMarkers(gray)
        # corners, ids, rejected = detector.detectMarkers(frame)

        # 5. 検出結果の描画
        if ids is not None:
            # 枠線を描画
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # コンソールにも検出されたIDを表示
            for i in range(len(ids)):
                print(f"Detected ID: {ids[i][0]}")

        # ウィンドウに表示
        cv2.imshow("ArUco Detection Test", frame)

        # 'q' キーが押されたら終了
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
