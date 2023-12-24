# KAU_AI_Planning
한국항공대학교 AI 플래닝에서 PDDL과 관련된 자료를 정리해 놓았습니다.   
PDDL WIKI에서 자세한 문법을 확인할 수 있습니다. 다만 FF Planner가 지원하는 범위가 매우 한정적이기 때문에 안되는 내용이 많습니다. PDDL만 주로 확인하고 PDDL 2.1 이상은 무시하셔도 될 것 같습니다.   
PDDL WIKI: https://planning.wiki/ref/pddl

## FF Planner
FF planner는 PDDL을 풀 수 있는 플래너의 일종입니다. 리눅스 환경에서만 사용할 수 있습니다. 추천하는 사용법은 WSL을 이용하는 것입니다. 설치 방법과 컴파일에 필요한 파일을 올려놓았습니다. 참고하여 진행하면 됩니다.   
이외에도 다양한 플래너가 있습니다. 리눅스에서 동작하는 것이 대부분이라 리눅스 사용법을 알고 있는 것이 좋습니다.

### 설치법
1. WSL을 사용하여 우분투를 설치한다.
https://learn.microsoft.com/ko-kr/windows/wsl/install
위 링크에 나온대로 하면 WSL2를 사용할 수 있을겁니다. 윈도우 터미널을 관리자 권한으로 실행 후 wsl --install을 진행하여 그대로 사용해도 되고 진행 후 다른 우분투 버전을 사용하고 싶다면 Microsoft store에서 원하는 Ubuntu를 설치하면 됩니다.   

2. FF-v2.3_source.zip 파일을 WSL의 우분투에 복사한다.
```
cp <FF-v2.3_source.zip이 있는 경로> <파일을 복사할 위치>
```
```
cp /mnt/c/Desktop/FF-v2.3_source.zip ~
```
리눅스에서 copy 명령어는 cp입니다. 컴파일에 필요한 파일을 zip파일로 만들었습니다. 저는 사용하기 편하게 홈 디렉토리로 복사했습니다.   

3. 필요한 것들을 설치한다 컴파일과 압축 해제에 필요한 것들입니다
```
sudo apt update && sudo apt upgrade
sudo apt install zip -y
sudo apt install flex -y
sudo apt install bison -y
sudo apt install gcc -y
sudo apt install vim -y
```
컴파일, 압축 해제 등에 필요한 것들을 다운로드 받습니다.

4. source폴더의 압축을 해제한다
```
unzip FF-v2.3_source.zip
```

5. 폴더에 진입한다
```
cd FF-v2.3
```

6. 컴파일을 진행한다
```
make
```

7. FF planner를 사용한다.
FF Planner 사용법
```
ff -o <Domain 파일 경로> -f <Problem 파일 경로>
```
예시
```
ff -o ~/domain/domain-02.pddl ~/problem/problem-02.pddl
```

FF Planner : http://www.ai.mit.edu/courses/16.412J/ff.html   
Metric Planner : https://fai.cs.uni-saarland.de/hoffmann/ff.html   
이외의 다른 플래너 : https://nms.kcl.ac.uk/planning/   

7-1 만약 FF planner가 실행되지 않는다면 실행 권한을 부여해봅니다
```
sudo chmod 777 ./ff
```

7-2 경로를 다 치기 귀찮다면 단축 명령어를 만들 수 있습니다.
```
sudo vi ~/.bashrc
```
먼저 bashrc 파일을 수정하게 vi나 편집기를 이용해 열어줍니다.
```
alias ff='~/FF-v2.3/ff'
```
Bashrc의 마지막 줄에 위 내용을 추가합니다.
```
source ~/.bashrc
```
추가한 것을 적용합니다.   

7-3 최신 WSL에서는 GUI를 기본으로 사용할 수 있습니다. GUI가 작동하지 않으면 다음 글을 참고해 보세요.   
https://learn.microsoft.com/ko-kr/windows/wsl/tutorials/gui-apps   
저는 구글 크롬 설치 이전까지 했을 때 GUI가 작동했습니다.   


----------------------------------
## Docker_Worker_Robot
항구에서 크레인과 트럭을 이용해 컨테이너를 옮기는 문제입니다. 해당 파일은 다른 깃허브에서 가져왔습니다만 수업시간에 배우는 내용으로 만든 것으로 확인했습니다. 폴더 안에 더 자세한 내용이 있습니다.
출처 사이트에서 02번이 Docker_Worker_Robot 예제입니다.   
출처 : https://github.com/hfoffani/pddl-lib/tree/main/examples-pddl

   -------------------------------------------------
## UC Berkeley 팩맨 프로젝트 (Proj1-search-python3)   
출처 : http://ai.berkeley.edu/project_overview.html   
Pacman Project 항목 중 Search에 있는 파일을 가지고 진행했습니다.   

### 팩맨 실행
```
python pacman.py
```
팩맨을 실행하려면 저것만 입력하면 됩니다. 그러면 기본 팩맨이 시작되며 플레이어가 직접 키보드로 움직여서 게임을 진행합니다.

### 에이전트 사용
```
python pacman.py --layout tinyMaze --pacman GoWestAgent
```
위 명령어를 통해 맵을 변경하고 에이전트를 사용할 수 있습니다. --layout은 -l로, --pacman은 -p로 줄여도 됩니다.    
- ```--layout``` : 인자는 반드시 layout폴더에 존재하는 파일명이어야 합니다. 기본값은 MediumClassic이다.   
- ```--pacman``` : 인자는 *Agents.py 파일 안에 클래스여야 합니다. 기본값은 KeyboardAgent이다.
- ```--ghost``` : ghost agent를 설정할 수 있다. 기본값은 RandomGhost이다.

### 제 방식
저는 일반 팩맨 이외에도 탐색, 한붓그리기 문제도 만들었습니다. 따라서 문제에 따라서 다른 PDDL파일을 만들고 플래너가 풀 게 했습니다.   

#### 한붓그리기
```
python pacman.py -l p4 -p FFSearchAgent -a prob='oneshot'
```
- ```-l``` : ```--layout``` 과 동일하며 맵을 지정한다. 기본값은 MediumClassic이다.   
- ```-p``` : ```--pacman``` 과 동일하며 사용할 에이전트를 지정한다. 기본값은 KeyboardAgent이다.   
- ```-a``` : 에이전트가 사용하는 인자값을 넘겨준다. 여러개를 넘겨줄때는 쉼표 ( , )를 통해 구분한다.   
위 명령어를 입력하면 플래너가 생성된 PDDL을 풀고 솔루션에 따라서 에이전트는 행동한다. 목표는 방문한 칸을 밟지 않으면서 모든 칸을 방문하는 것이다.   

#### Search
```
python pacman.py -l mediumMaze -p FFSearchAgent -a prob='search'
```
인자는 위에서 설명했으니 넘어가겠다. 단순 탐색 문제를 풀며 고스트는 없는 것으로 가정하고 작성했다.   
목표는 모든 칸에 음식이 없게 하는 것이다.   

#### Pacman
```
python pacman.py -p FFSearchAgent -a prob='pacman',cycle=2
```
- ```cycle``` : 플래너를 실행하는 주기이다. 기본값은 1이다.   
일반 팩맨을 푸는 에이전트이다. 위의 문제들과 다른 것은 액션을 선택하는 순간마다 플래너를 돌려서 새로 솔루션을 만들어낸다. cycle을 통해 주기를 조절할 수 있다.
추가로 목표를 생성할 때 처음에는 Search와 동일하게 '모든 음식을 먹기'로 했으나 자주 죽어서 '맨해튼 거리로 유령에서 가장 먼곳에 있는 음식 먹기'로 수정하였다. 이런 방식을 통해 죽는 빈도가 줄었다.
자세한 구현은 SearchAgent.py에 있는 FFSearchAgent 클래스를 참고하세요.
   
각 문제에 해당하는 PDDL 파일도 첨부되어 있으니 참고하면 좋습니다. oneshot과 search는 맨 처음에만 만들기 때문에 첨부된 파일의 결과를 모두 사용합니다. 다만 pacman은 매 Action마다 새 파일을 만들기 때문에 첨부된 파일은 생성된 파일의 일부입니다.
      
나중에 이 글을 참고하는 학생이 있다면 유령과의 거리에 따라서 목표를 다르게 설정하는 방식을 추가하면 죽는 빈도를 더 줄일 수 있을 것이다.
그리고 유령 에이전트도 추가하면 멀티에이전트 환경을 구현할 수 있다. 기본값은 RandomGhost라서 가능한 액션 중에서 Uniform한 분포로 선택한다.

