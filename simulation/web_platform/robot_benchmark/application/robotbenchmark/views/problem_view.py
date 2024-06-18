from django_filters.rest_framework import DjangoFilterBackend
from drf_spectacular.utils import extend_schema, extend_schema_view
from rest_framework import viewsets, status
from rest_framework.filters import OrderingFilter
from rest_framework.permissions import IsAuthenticated, IsAdminUser

from ..filters.problem_filter import ProblemFilter
from ..models import Problem
from ..serializers.problem_serializer import ProblemSerializer


@extend_schema(tags=["problems"])
@extend_schema_view(
    retrieve=extend_schema(
        summary="Детальная информация о задаче",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    list=extend_schema(
        summary="Получение списка задач",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    update=extend_schema(
        summary="Обновление данных о задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    create=extend_schema(
        summary="Создание задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    destroy=extend_schema(
        summary="Удаление задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    partial_update=extend_schema(
        summary="Обновление с необязательными полями задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    )
)
class ProblemViewSet(viewsets.ModelViewSet):
    queryset = Problem.objects.all()
    serializer_class = ProblemSerializer
    filter_backends = [DjangoFilterBackend, OrderingFilter]
    filterset_class = ProblemFilter

    def get_permissions(self):
        if self.request.method in ['GET']:
            # Для запросов GET требуется аутентификация пользователя
            permission_classes = [IsAuthenticated]
        else:
            # Для запросов POST, PUT и DELETE требуется быть суперпользователем
            permission_classes = [IsAdminUser]
        return [permission() for permission in permission_classes]
