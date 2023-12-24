# Docker Worker Robot
PDDL을 배울 때 대표적으로 배우는 문제입니다. 문제상황은 항구에서 컨테이너를 옮기는 상황입니다. 우리의 목표는 크레인과 로봇을 이용해 모든 컨테이너를 반대편으로 옮기는 것입니다.   
PDDL은 Domain과 Problem 파일로 나눠지며 둘을 함께 사용해야 플래너가 문제를 풀 수 있습니다.   
작성했던 보고서의 일부도 첨부하였으니 참고하세요.

## Domain
도메인은 문제에서 사용하는 predicate과 action을 모아놓은 것입니다. 이제 코드에서 주로 사용하는 것을 설명하겠습니다.   

- ```domain``` : 도메인 명을 지정합니다. 도메인 명을 통해 problem파일과 연동할 수 있기 때문에 반드시 일치시켜야 합니다.
- ```requirements``` : C언어의 include나 파이썬의 import와 비슷한 개념입니다. 추가기능을 사용할 수 있게 해줍니다. 자주 사용하는 기능은 ```:strips``` 와 ```:typing``` 입니다. WIKI를 통해 다양한 기능을 확인할 수 있지만 플래너에 따라 사용하지 못하는 것이 많습니다.
- ```types``` : 클래스와 비슷한 것입니다. 문제에 존재하는 것을 집어넣으면 됩니다.
- ```predicate``` : 술어라고 부르며 관계나 논리를 표현하는데 사용합니다. 문제의 state를 표현하는데 사용됩니다.
- ```action``` : 행동을 표현합니다. 액션을 통해 State를 변화시킬 수 있습니다. (State Transition)

### Action
위에서 언급한대로 액션을 통해서 State에 존재하는 predicate를 제거하거나 추가하여 State를 변화시키는 State Transition을 일으킵니다. 액션은 액션명, parameters, precondition, effect로 이뤄집니다.

- ```parameters``` : 액션의 인자값입니다.
- ```precondition``` : 액션이 발생하기 위해 필요한 전제조건입니다. 해당 조건을 만족하지 못하면 액션을 실행할 수 없습니다.
- ```effect``` : 액션의 결과입니다. State에 긍정 predicate는 추가되며 부정(not) predicate는 제거됩니다.


해당 문제에 대한 액션들을 그림으로도 표현해 놓았으니 참고하세요.
#### move
![move](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/move.png)
#### put
![put](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/put.png)
#### take
![take](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/take.png)
#### load
![load](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/load.png)
#### unload
![unload](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/unload.png)


## Problem
Problem은 실제 문제를 표현하는 것입니다. objects, init, goal로 구성됩니다.

- ```problem``` : 문제명입니다. 아무거나 입력해도 됩니다.
- ```domain``` : 이 문제가 어느 도메인에 속하는지를 나타냅니다. 이 부분을 통해 domain파일과 연결하기 때문에 일치하지 않으면 플래너에서 에러를 발생합니다.
- ```objects``` : 문제에 존재하는 인스턴스입니다. 인스턴스들을 입력한 후 마지막에 타입을 표현해줍니다.
- ```init``` : 문제의 초기상태를 나타냅니다. domain에서 정의한 predicate를 통해 문제를 표현합니다.
- ```goal``` : 문제의 목표상태를 나타냅니다. 이 문제에서는 어떤 컨테이너가 어디에 위치하는지를 나타냅니다.

아래에서 초기 상태와 목표 상태를 확인할 수 있습니다. 목표 상태는 goal에 명시된 predicate가 state에 모두 존재하기만 하면 달성합니다. goal로 정의된 것은 컨테이너가 어느 pile에만 있으면 된다이기 때문에 컨테이너의 쌓인 순서가 그림과는 달라질 수 있습니다.
![init](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/init.png)
![goal](https://github.com/kjh101402/KAU_AI_Planning/blob/main/Docker_Worker_Robot/img/goal.png)
