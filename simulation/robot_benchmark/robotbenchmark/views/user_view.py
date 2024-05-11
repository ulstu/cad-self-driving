from django.contrib.auth import get_user_model
from drf_spectacular.utils import extend_schema, extend_schema_view
from rest_framework import status
from rest_framework.viewsets import ModelViewSet

from ..filters.user_filter import UserFilter
from ..permissions import UserPermission
from ..serializers.user_serializer import UserSerializer

UserModel = get_user_model()


@extend_schema(tags=["users"])
@extend_schema_view(
    retrieve=extend_schema(
        summary="Детальная информация пользователе",
        responses={
            status.HTTP_200_OK: UserSerializer
        }
    ),
    list=extend_schema(
        summary="Получение списка пользователей",
        responses={
            status.HTTP_200_OK: UserSerializer
        }
    ),
    update=extend_schema(
        summary="Обновление данных о пользователе",
        responses={
            status.HTTP_200_OK: UserSerializer
        }
    ),
    create=extend_schema(
        summary="Создание пользователя",
        responses={
            status.HTTP_200_OK: UserSerializer
        }
    ),
    destroy=extend_schema(
        summary="Удаление пользователя",
        responses={
            status.HTTP_200_OK: UserSerializer
        }
    ),
    partial_update=extend_schema(
        summary="Обновление с необязательными полями пользователей",
        responses={
            status.HTTP_200_OK: UserSerializer
        }
    )
)
class UserViewSet(ModelViewSet):
    serializer_class = UserSerializer
    queryset = UserModel.objects.all().order_by("-date_joined")
    permission_classes = [UserPermission, ]
    filterset_class = UserFilter

