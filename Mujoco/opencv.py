import cv2

# 얼굴 인식용 Haar Cascade 로드
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

cap = cv2.VideoCapture(0) # 0번 : 아이폰 , 1번 : mac 

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break

    # 그레이스케일 변환 (검출 성능 향상)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 얼굴 검출
    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
    )

    # 검출된 얼굴 위치에 사각형 그리기
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 결과 화면 출력
    cv2.imshow("Face Detection", frame)

    # ESC(27) 키 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()