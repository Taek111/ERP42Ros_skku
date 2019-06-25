# Report 20190621 result

- 작성자 : 가태권
- 작성일시 : 2019-06-21

- 실험일시 : 2019-06-21 21:30 - 22:30
- 날씨 : 맑음
- 참여인원 : 2명
  - 조장 : 오일호
  - 조원 : 가태권 

## 실험 목표
- ERP42위에 UPPER 컨트롤러로 쓰일  Jetson TX2를 부착하고 전원과 변압기를 통해 전원을 연결하여  제대로 작동하는지 확인한다. 
- Jetson TX2와 PCU를 연결하고 ERP42Ros 노드를 실행하여 UPPER와 PCU간 RS232통신을 확인한다. 
- Auto mode를 구동함에 있어서 의도치 않은 에러가 있는지 확인한다. 

## 실험 과정
1. 사전보고서의 내용대로 로봇을 리프트업 함  
1. ERP42에 Jetson TX2과 거치대를 부착하고 체크리스트의 순서에 따라 로봇과 Jetson의 전원을 켬. 
1. Manual mode에서 전진, 후진, 조향, 브레이크 등의 기능이 동작하는 것을 확인한 후 Auto mode로 바꿈.
1. Jetson에서 ERP42Ros 노드 실행 후 결과 확인 
1. 리프트 다운 및 정리 

## 실험 결과 
1.Auto mode에서 ERP42Ros실행 결과 로봇의 시스템패널에 Auto_Fail이 출력되고,  Jetson은 50Hz의 통신주기를 맞추지 못하며, 패킷 쓰기 동작은 문제없이 실행되었으나  패킷 읽기에서 에러가 발생함. 또한 통신 데이터 로그 생성에도 실패함.

## 새로 알게된 사실
1. PCU와 UPPER 컨트롤러와의 통신은 Auto mode 뿐만 아니라 Manual mode와 Auto_fail에서도 이루어짐.
2. 실험 당시 Jetson에서 PCU를 인식할 때 두 장치 사이의 USB-to-RS232 cable이 Prolific USB-to-Serial Bridge라는 이름으로 ttyUSB0 포트에 인식됨.  따라서 실행 커맨드는 "rosrun ERP42Ros /dev/ttyUSB0"가 됨. 

## 고찰
1. 매뉴얼에 따르면,  로봇이 Auto_Fail이 된 이유에는  Upper 컨트롤러에서 PCU로 전달된 패킷의 AM이 0x00이거나, 물리적인 단선 통신의 에러이거나, Upper-PCU 간 Alive counter에 1초동안 정확히 들어오지 않는 것, 이렇게 세 가지의 상황이 있다.  이번 실험에서 로봇이 Auto mode로 자동적으로 넘어가게 하지 않기 위해서 Upper-to-PCU 패킷의 AM을 0x00으로 두었는데,  따로 target message가 없어 AM의 값이 변하지 않고 0x00으로 PCU에 전달되서 Auto_fail의 직접적인 원인중 하나가 되었다.  또한 PCU-to-Upper 패킷의 읽기가 제대로 수행되지 않은 것으로 보아 Alive counter에서의 오류 또한 Auto_fail의 원인이 될 수 있다. 
1. 실험에서 발생한 오류를 고치기 위해 우선, Upper-to-PCU 패킷의 AM을 0x01로 초기화하고, RS232포트 또한 드라이버를 이용해 결속을 단단히 한다. 그리고 통신 읽기 오류를 고치기 위해 패킷별 리틀엔디안, 빅엔디안을 다시 한번 확인하고 라즈베리파이의 시리얼통신을 이용해 패킷 읽기 및 로그 생성 과정을 미리 실험해 본다. 
