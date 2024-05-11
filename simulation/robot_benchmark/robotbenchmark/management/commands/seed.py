from django.contrib.auth.models import User
from django.core.management.base import BaseCommand, CommandError
from robotbenchmark.models import Problem


def createProblem(params):
    if Problem.objects.filter(title=params['title']).count() == 0:
        problem = Problem()
        problem.title = params['title']
        problem.world_path = params['world_path']
        problem.difficulty = params['difficulty']
        problem.description = params['description']
        problem.author = params['user']
        problem.save()


class Command(BaseCommand):
    help = 'Add robotbenchmark problems'

    def handle(self, *args, **options):
        user = User.objects.first()

        # 1
        createProblem({
            'title': 'A + B',
            'world_path': 'test_simulation/worlds/test_simulation.wbt',
            'image_path': 'robotbenchmark/img/Thymio2.png',
            'difficulty': 10,
            'description': 'Learn how to program robot controller',
            'user': user
        })

        # 2
        createProblem({
            'title': 'Robot Programming',
            'world_path': 'robot_programming/worlds/robot_programming.wbt',
            'image_path': 'robotbenchmark/img/Thymio2.png',
            'difficulty': 15,
            'description': 'Learn how to program a robot, save your controller program, revert and run the simulation.',
            'user': user
        })

        # 3
        createProblem({
            'title': 'Obstacle Avoidance',
            'world_path': 'obstacle_avoidance/worlds/obstacle_avoidance.wbt',
            'image_path': 'robotbenchmark/img/Thymio2.png',
            'difficulty': 20,
            'description': 'Program a Thymio II robot to cross a classroom filled with obstacles as quickly as possible.',
            'user': user
        })

        # 4
        createProblem({
            'title': 'Square Path',
            'world_path': 'square_path/worlds/square_path.wbt',
            'image_path': 'robotbenchmark/img/Pioneer3dx.png',
            'difficulty': 25,
            'description': 'Program a Pioneer 3-DX robot to follow a 2m x 2m square trajectory, quickly and precisely.',
            'user': user
        })

        # 5
        createProblem({
            'title': 'Inverted Pendulum',
            'world_path': 'inverted_pendulum/worlds/inverted_pendulum.wbt',
            'image_path': 'robotbenchmark/img/E-Puck.png',
            'difficulty': 30,
            'description': 'Program an e-puck robot to maintain an inverted pendulum up as long as possible.',
            'user': user
        })

        # 6
        createProblem({
            'title': 'Pick And Place',
            'world_path': 'pick_and_place/worlds/pick_and_place.wbt',
            'image_path': 'robotbenchmark/img/Youbot.png',
            'difficulty': 35,
            'description': 'Program a youBot mobile manipulator robot to pick and place a cube as quickly as possible.',
            'user': user
        })

        # 7
        createProblem({
            'title': 'Pit Escape',
            'world_path': 'pit_escape/worlds/pit_escape.wbt',
            'image_path': 'robotbenchmark/img/BB-8.png',
            'difficulty': 40,
            'description': 'Program a BB-8 robot lost in a sand desert to climb out of a pit as quickly as possible.',
            'user': user
        })

        # 8
        createProblem({
            'title': 'Wall Following',
            'world_path': 'wall_following/worlds/wall_following.wbt',
            'image_path': 'robotbenchmark/img/Pioneer3dx.png',
            'difficulty': 45,
            'description': 'Program a Pioneer 3-DX robot to follow a random wall on its left-hand side, quickly and precisely.',
            'user': user
        })

        # 9
        createProblem({
            'title': 'Highway Driving',
            'world_path': 'highway_driving/worlds/highway_driving.wbt',
            'image_path': 'robotbenchmark/img/LincolnMKZ.png',
            'difficulty': 50,
            'description': 'Program a Lincoln MKZ autonomous car to drive as fast as possible on a crowded highway.',
            'user': user
        })

        # 10
        createProblem({
            'title': 'Humanoid Sprint',
            'world_path': 'humanoid_sprint/worlds/humanoid_sprint.wbt',
            'image_path': 'robotbenchmark/img/Nao.png',
            'difficulty': 55,
            'description': 'Program a Aldebaran NAO humanoid robot to walk as quickly as possible on a 10 meters race.',
            'user': user
        })

        # 11
        createProblem({
            'title': 'Humanoid Marathon',
            'world_path': 'humanoid_marathon/worlds/humanoid_marathon.wbt',
            'image_path': 'robotbenchmark/img/RobotisOp2.png',
            'difficulty': 60,
            'description': 'Program a Robotis OP2 humanoid robot to walk as far as possible before its battery runs out of energy.',
            'user': user
        })

        # 12
        createProblem({
            'title': 'Visual Tracking',
            'world_path': 'visual_tracking/worlds/visual_tracking.wbt',
            'image_path': 'robotbenchmark/img/AiboErs7.png',
            'difficulty': 65,
            'description': 'Program an Aibo robot to move its head to track a rubber duck moving in a scattered environment.',
            'user': user
        })

        # 13
        createProblem({
            'title': 'Viewpoint Control',
            'world_path': 'viewpoint_control/worlds/viewpoint_control.wbt',
            'image_path': 'robotbenchmark/img/Thymio2.png',
            'difficulty': 5,
            'description': 'Learn how to control the position and orientation of the viewpoint in the 3D world.',
            'user': user
        })
        self.stdout.write(self.style.SUCCESS('Success'))
