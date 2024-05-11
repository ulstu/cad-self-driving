from django.contrib.auth.models import User
from django.test import TestCase, Client
from rest_framework_simplejwt.tokens import AccessToken

from robotbenchmark.models import Problem, CommandQueue


class ProblemUserTestCase(TestCase):
    """Тесты для проверки работоспособности создания задачи пользователю"""

    c = Client()
    CREATE_PROBLEM_URL = "/api/users-problem/"
    GET_COMMAND_URL = "/api/commands/"

    def setUp(self):
        self.user = User.objects.create_user(username='test_for_task', password='abcdefasd', email="task@mail.ru")
        self.problem = Problem.objects.create(
            title="test problem",
            description="string",
            world_path="usr/test/mock/path",
            difficulty=1,
            author=self.user
        )
        self.token = AccessToken.for_user(self.user)

    def test_create_problem_user(self):
        """Тест на создание задачи пользователю"""
        self.c.login(username='test_for_task', password='abcdefasd')
        print(f"Отправляем запрос на {self.CREATE_PROBLEM_URL}")
        response = self.c.post(
            f"{self.CREATE_PROBLEM_URL}",
            {
                "is_completed": True,
                "points": 0,
                "user": self.user.id,
                "problem": self.problem.id
            },
            HTTP_AUTHORIZATION=f'Bearer {self.token}',
            content_type="application/json",
        )
        print(f"Проверяем статус ответа {response.status_code=} == 201")
        assert response.status_code == 201

        print(f"Проверяем что команда на создание контейнеров запушилась в очередь...")
        check_command = CommandQueue.objects.all()

        print(f"Команд в очереди: {len(check_command)} > 0")
        assert len(check_command) > 0

        print(f"Делаем запрос на исполнение команды: {self.GET_COMMAND_URL}")
        response = self.c.get(self.GET_COMMAND_URL)
        assert response.status_code == 200
        check_command = CommandQueue.objects.all()

        print(f"Проверяем что команда на поднятие контейнера исполнилась: команд в очереди {len(check_command)} == 0")
        assert len(check_command) == 0
