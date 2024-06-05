카메라의 화면을 그레이스케일 변환, 밝기보정, 이진화와 모폴로지변환 후 레이블링 함수를 이용해 라인후보 영역을 찾아내어 알고리즘을 통해 진짜 라인을 찾아 라인사이 가운데를 따라 이동하는 로봇

's'키를 누르면 출발, 'ctrl+c'를 누르면 종료 

[알고리즘] 
로봇의 초기위치는 항상 라인사이 가운데에 있다고 가정함 

처음 영상의 가운데지점과(로봇위치) 가장 가까운 객체의 무게중심을 찾아 그것을 진짜 라인으로 인식, 무게중심을 저장함

프레임마다 저장해둔 무게중심과 객체들의 무게중심을 비교하여 가장 가까운 무게중심을 진짜 라인으로 인식, 무게중심을 갱신 

라인이 사라지는 경우를 대비하여 진짜 라인에서 일정범위안에있는 객체만 찾도록 하였음(없으면 무게중심 갱신안함)

영상의 가운데지점 - 진짜 라인의 무게중심 을 에러값으로 설정하여 직진속도100으로 다이나믹셀 속도를 조정했음 

좌측속도명령 = 직진속도(100) - 게인 * error 

우측속도명령 = 직진속도(100) - 게인 * error
