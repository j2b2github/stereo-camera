# stereo camera를 활용한 거리 추정

2차원 영상에서는 동일한 물체의 길이나 면적을 구하여도 카메라로 부터의 거리에 따라서 길이나 면적이 달라진다.
stereo camera를 활용하여 거리 정보를 구하면 거리의 변화에도 동일한 물체에 대해서 동일한 길이와 면적을 구할 수 있다.


## 순서
1. capture_images.cpp
    스테레오 캘리브레이션을 위한 이미지 캡쳐
2. 스테레오 캘리브레이션
    calibrate.cpp
3. Z축 캘리브레이션을 위한 단위 거리별 이미지 캡쳐
    estimateZ.cpp, Z축 캘리브레이션 모드
4. Z축 캘리브레이션
    calibrateZ.cpp
5. Z축 추정
    estimateZ.cpp, 스테레오 카메라 모드


## 기타
* common.hpp
    공통으로 사용하는 함수
* record_stereo_cideo.cpp
    스테레오 영상 저장용
