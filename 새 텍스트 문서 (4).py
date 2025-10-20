import cv2
from ultralytics import YOLO
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import time

# --- 사용자가 반드시 수정해야 할 부분 ---

# Tapo TC60/C200 등 IP 카메라의 RTSP URL 설정
# 1. Tapo 앱에서 RTSP 기능을 활성화하고, 별도의 '장치 계정'(Username/Password)을 생성해야 합니다.
#    (이 계정은 Tapo 로그인 계정과 다릅니다!)
# 2. 아래 변수들을 사용자 환경에 맞게 변경하세요.
TAPO_USER = 'name'   # Tapo 앱에서 만든 카메라 장치 계정 ID
TAPO_PASS = 'key'   # 위 계정 비밀번호
TAPO_IP = 'IP'   # 라우터에서 확인한 카메라 IP (예: 192.168.1.100)
STREAM = 'stream1'       # 'stream1' (고화질) 또는 'stream2' (저화질)
RTSP_PORT = 554          # Tapo 기본 포트

# 완성된 RTSP URL (이 형식을 사용해야 합니다)
RTSP_URL = f"rtsp://{TAPO_USER}:{TAPO_PASS}@{TAPO_IP}:{RTSP_PORT}/{STREAM}"

# 영상 저장 설정
# ************ 📢 저장 경로 및 파일명 ************
# 저장을 원치 않을 경우 이 경로를 None으로 설정하거나 주석 처리하세요.
OUTPUT_PATH = "output_detected_video.mp4" 
RECORDING_FPS = 20.0 # 저장될 영상의 초당 프레임 수 (안정적인 녹화를 위해 고정 값 사용)
# **********************************************

# 한글 폰트 설정 경로 (사용자 시스템 환경에 맞게 수정하세요. Windows의 경우)
font_path = "C:/Windows/Fonts/malgun.ttf"

# ----------------------------------------

# 폰트 로드
try:
    font = ImageFont.truetype(font_path, 30)
except IOError:
    print(f"경고: {font_path} 폰트를 찾을 수 없어 기본 폰트를 사용합니다.")
    font = ImageFont.load_default()

# YOLO 모델 로드 (yolov5n.pt)
try:
    model = YOLO("yolov5n.pt")
except Exception as e:
    print(f"YOLO 모델 로드 실패: {e}")
    exit()

# 연결 함수 정의 (재연결 로직을 분리하여 안정성 향상)
def connect_rtsp(url, attempts=3):
    print(f"RTSP 스트림 연결 시도: {url}")
    for attempt in range(attempts):
        # cv2.CAP_FFMPEG 백엔드 사용은 RTSP 연결 안정성을 높여줍니다.
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            # 지연을 줄이기 위해 버퍼 크기 설정 (OpenCV가 지원하는 경우)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print("성공적으로 연결되었습니다.")
            return cap
        print(f"연결 실패 (시도 {attempt + 1}/{attempts}). 1초 후 재시도...")
        time.sleep(1)
    return None

# 1. IP 카메라 연결 시도
cap = connect_rtsp(RTSP_URL)

if cap is None:
    print("\n[치명적인 오류] 카메라 스트림을 열 수 없습니다.")
    print("RTSP URL, IP 주소, 장치 계정 정보를 다시 확인하고 방화벽 설정을 체크하십시오.")
    exit()

# 2. 영상 저장 설정 (VideoWriter 초기화)
out = None
if OUTPUT_PATH:
    # 캡처 객체에서 실제 프레임 크기를 가져옵니다.
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # MP4V 코덱 설정
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    
    try:
        out = cv2.VideoWriter(OUTPUT_PATH, fourcc, RECORDING_FPS, (width, height))
        print(f"✅ 영상 저장을 시작합니다: {OUTPUT_PATH} ({width}x{height} @ {RECORDING_FPS} FPS)")
    except Exception as e:
        print(f"❌ 경고: VideoWriter 초기화 실패. 영상 저장이 되지 않습니다. 오류: {e}")
        out = None


# 실시간 상태 업데이트 설정
UPDATE_INTERVAL_SEC = 1.0 
last_update_time = time.time()
person_count_status = 0
status_text_display = "대기 중"
current_persons = 0

# 상태 임계값 설정
def get_status(count):
    if count >= 15:
        return "많음"
    elif 8 <= count <= 14:
        return "중간"
    else:
        return "적음"

print("스트리밍 시작...")
while True:
    # 3. 프레임 읽기
    ret, frame = cap.read()
    
    if not ret or frame is None:
        # 프레임 읽기 실패 시 재연결 시도
        print("경고: 프레임 읽기 실패. 재연결을 시도합니다...")
        cap.release()
        cap = connect_rtsp(RTSP_URL, attempts=3) # 3회 재시도
        if cap is None:
            print("[치명적인 오류] 재연결 실패. 프로그램을 종료합니다.")
            break
        continue

    # 4. YOLO 추론 (stream=True로 메모리 효율 개선)
    results = model(frame, verbose=False, stream=True)
    
    current_persons = 0
    
    # PIL 이미지 변환 (한글 폰트 사용을 위해)
    frame_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(frame_pil)

    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        names = result.names
        
        for i, cls in enumerate(classes):
            class_name = names[int(cls)]
            
            if class_name == "person":
                current_persons += 1
                x1, y1, x2, y2 = boxes[i]
                
                # 경계 박스 그리기 (색상 변경: 마젠타)
                draw.rectangle([x1, y1, x2, y2], outline="#FF00FF", width=3) 
                
                # 라벨 텍스트 (한글)
                label = f"사람 {confs[i]:.2f}"
                draw.text((x1, max(0, y1 - 35)), label, font=font, fill="#FF00FF")

    # 5. 상태 갱신 (1초마다)
    current_time = time.time()
    if current_time - last_update_time >= UPDATE_INTERVAL_SEC:
        person_count_status = current_persons
        status_text_display = get_status(person_count_status)
        last_update_time = current_time

    # 사람 수 상태 표시 (텍스트 색상: 노란색)
    status_text = f"현재 감지 인원: {current_persons}명 | 상태: {status_text_display}"
    
    # 배경 추가 (텍스트 가독성 향상)
    text_bbox = draw.textbbox((20, 20), status_text, font=font)
    draw.rectangle([text_bbox[0] - 5, text_bbox[1] - 5, text_bbox[2] + 5, text_bbox[3] + 5], fill=(0, 0, 0, 180)) 
    
    draw.text((20, 20), status_text, font=font, fill="yellow")

    # 6. 다시 OpenCV 이미지로 변환
    frame = cv2.cvtColor(np.array(frame_pil), cv2.COLOR_RGB2BGR)

    # 7. 영상 저장 (VideoWriter가 정상 초기화된 경우에만)
    if out is not None:
        out.write(frame)

    # 화면에 출력
    cv2.imshow("YOLO 실시간 사람 수 탐지 (TP-Link TC60)", frame)

    # 8. 키보드 입력 처리
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        # 스페이스바로 일시정지
        print("일시 정지...")
        while True:
            key2 = cv2.waitKey(0) & 0xFF
            if key2 == ord(' '):
                print("재생 계속...")
                break
            elif key2 == ord('q'):
                cap.release()
                if out is not None:
                    out.release()
                    print(f"\n✨ 저장 완료! 감지 영상이 다음 위치에 저장되었습니다: {OUTPUT_PATH}")
                cv2.destroyAllWindows()
                exit()
            
# 9. 리소스 해제
cap.release()
if out is not None:
    out.release()
    print(f"\n✨ 저장 완료! 감지 영상이 다음 위치에 저장되었습니다: {OUTPUT_PATH}")
cv2.destroyAllWindows()

