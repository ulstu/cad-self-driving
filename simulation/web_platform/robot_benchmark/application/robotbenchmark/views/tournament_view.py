from django_filters.rest_framework import DjangoFilterBackend
from drf_spectacular.utils import extend_schema, extend_schema_view
from rest_framework import viewsets, status
from rest_framework.filters import OrderingFilter
from rest_framework.permissions import IsAuthenticated, IsAdminUser

from ..models import Tournament
from ..serializers.tournament_serializer import TournamentSerializer


@extend_schema(tags=["tournament"])
@extend_schema_view(
    retrieve=extend_schema(
        summary="Детальная информация о соревновании",
        responses={
            status.HTTP_200_OK: TournamentSerializer
        }
    ),
    list=extend_schema(
        summary="Получение списка соревнований",
        responses={
            status.HTTP_200_OK: TournamentSerializer
        }
    ),
    update=extend_schema(
        summary="Обновление данных о соревновании",
        responses={
            status.HTTP_200_OK: TournamentSerializer
        }
    ),
    create=extend_schema(
        summary="Создание соревнования",
        responses={
            status.HTTP_200_OK: TournamentSerializer
        }
    ),
    destroy=extend_schema(
        summary="Удаление соревнования",
        responses={
            status.HTTP_200_OK: TournamentSerializer
        }
    ),
    partial_update=extend_schema(
        summary="Обновление с необязательными полями соревнования",
        responses={
            status.HTTP_200_OK: TournamentSerializer
        }
    )
)
class TournamentViewSet(viewsets.ModelViewSet):
    """ViewSet Соревнования"""
    queryset = Tournament.objects.all()
    serializer_class = TournamentSerializer
    filter_backends = [DjangoFilterBackend, OrderingFilter]

    def get_permissions(self):
        if self.request.method in ['GET']:
            # Для запросов GET требуется аутентификация пользователя
            permission_classes = [IsAuthenticated]
        else:
            # Для запросов POST, PUT и DELETE требуется быть суперпользователем
            permission_classes = [IsAdminUser]
        return [permission() for permission in permission_classes]
