# stereo camera를 활용한 거리 추정
2차원 영상에서는 동일한 물체의 길이나 면적을 구하여도 카메라로 부터의 거리에 따라서 길이나 면적이 달라진다.<br/>
stereo camera를 활용하여 거리 정보를 구하면 거리의 변화에도 동일한 물체에 대해서 동일한 길이와 면적을 구할 수 있다.<br/><br/>

## 환경
* setup_environment.sh<br/><br/>

## 순서
1. capture_images.cpp<br/>
    스테레오 캘리브레이션을 위한 이미지 캡쳐<br/>
2. calibrate.cpp<br/>
    스테레오 캘리브레이션<br/>
3. estimateZ.cpp<br/>
    Z축 캘리브레이션을 위한 단위 거리별 이미지 캡쳐<br/>
    settings/default.yml을 열어서 estimateZ_mode: 2(Z축 캘리브레이션 모드)로 수정 후 실행<br/>
4. calibrateZ.cpp<br/>
    Z축 캘리브레이션<br/>
5. estimateZ.cpp<br/>
    거리 추정, 거리에 따른 원의 지름과 면적 측정<br/>
    settings/default.yml을 열어서 estimateZ_mode: 1(스테레오 카메라 모드)로 수정 후 실행<br/><br/>

## 기타
* common.hpp<br/>
    공통으로 사용하는 함수<br/>
* record_stereo_cideo.cpp<br/>
    스테레오 영상 저장용<br/><br/>

## 데모 영상
https://youtu.be/H-9846qvp3w
