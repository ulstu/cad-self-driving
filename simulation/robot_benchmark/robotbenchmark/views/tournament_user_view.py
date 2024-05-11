from django_filters.rest_framework import DjangoFilterBackend
from drf_spectacular.utils import extend_schema, extend_schema_view
from rest_framework import viewsets, status
from rest_framework.filters import OrderingFilter
from rest_framework.permissions import IsAuthenticated

from robotbenchmark.serializers.tournament_user_serializer import TournamentUserSerializer
from robotbenchmark.models import TournamentUser


@extend_schema(tags=["users-tournament"])
@extend_schema_view(
    retrieve=extend_schema(
        summary="Детальная информация о соревнованиях пользователя",
        responses={
            status.HTTP_200_OK: TournamentUserSerializer
        }
    ),
    list=extend_schema(
        summary="Детальная информация о всех соревнованиях пользователей",
        responses={
            status.HTTP_200_OK: TournamentUserSerializer
        }
    ),
    update=extend_schema(
        summary="Обновление данных о соревновании пользователя",
        responses={
            status.HTTP_200_OK: TournamentUserSerializer
        }
    ),
    create=extend_schema(
        summary="Создание соревнования пользователю",
        responses={
            status.HTTP_200_OK: TournamentUserSerializer
        }
    ),
    destroy=extend_schema(
        summary="Удаление соревнования пользователю",
        responses={
            status.HTTP_200_OK: TournamentUserSerializer
        }
    ),
    partial_update=extend_schema(
        summary="Обновление с необ. полями соревнования пользователю",
        responses={
            status.HTTP_200_OK: TournamentUserSerializer
        }
    )
)
class TournamentUserViewSet(viewsets.ModelViewSet):
    queryset = TournamentUser.objects.all()
    serializer_class = TournamentUserSerializer
    filter_backends = [DjangoFilterBackend, OrderingFilter]
    permission_classes = [IsAuthenticated]
