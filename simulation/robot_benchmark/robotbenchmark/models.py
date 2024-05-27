from django.contrib.auth.models import User
from django.core.validators import MinValueValidator, MaxValueValidator
from django.db import models

from robotbenchmark.enums.webots_ros2_projects_dir import WebotsRosProjects


class Problem(models.Model):
    """Модель задачи"""
    title = models.CharField(max_length=300)
    description = models.CharField(max_length=1000, null=True)
    world_path = models.CharField(choices=WebotsRosProjects.get_choices())
    image = models.ImageField(upload_to='images/', default="default_img.jpg", null=True, blank=True)
    difficulty = models.FloatField()
    author = models.ForeignKey(User, related_name='problems', on_delete=models.DO_NOTHING)
    users = models.ManyToManyField(User, related_name='user_problems', through="ProblemUser")

    def __str__(self):
        return self.title


class ProblemUser(models.Model):
    """Многие ко многим Пользователь-Задача"""
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    problem = models.ForeignKey(Problem, on_delete=models.CASCADE)
    is_completed = models.BooleanField(null=False, default=False)
    points = models.IntegerField(default=0)
    robot_panel_port = models.IntegerField(validators=[MinValueValidator(10000), MaxValueValidator(12000)])
    vs_port = models.IntegerField(validators=[MinValueValidator(10000), MaxValueValidator(12000)])
    webots_stream_port = models.IntegerField(validators=[MinValueValidator(10000), MaxValueValidator(12000)])

    class Meta:
        constraints = [
            models.UniqueConstraint(fields=['robot_panel_port', 'vs_port', 'webots_stream_port'], name='unique_ports')
        ]

    def __str__(self):
        return self.user.username + " - " + self.problem.title


class Tournament(models.Model):
    """Модель Соревнования"""
    problems = models.ManyToManyField(Problem, related_name='tournaments')
    users = models.ManyToManyField(User, related_name='user_tournaments', through="TournamentUser")
    name = models.CharField(max_length=255, null=False, blank=False)
    description = models.CharField(max_length=5000)
    date_start = models.DateTimeField(auto_now_add=True)
    date_end = models.DateTimeField()

    def __str__(self):
        return self.name


class TournamentUser(models.Model):
    """Многие ко многим Пользователь-Соревнование"""
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    tournament = models.ForeignKey(Tournament, on_delete=models.CASCADE)
    is_completed = models.BooleanField(null=False, default=False)
    points = models.IntegerField(default=0)

    def __str__(self):
        return self.user.username + " - " + self.tournament.name


class CommandQueue(models.Model):
    """
    Модель для очереди задач, которые необходимо выполнить на хостовой машине.
    ! Лучше в будущем заменить на брокер сообщений !
    """
    command = models.CharField()
