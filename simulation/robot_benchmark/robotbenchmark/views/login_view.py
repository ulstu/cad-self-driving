from django.contrib.auth import login
from drf_spectacular.utils import extend_schema_view, extend_schema
from rest_framework import status
from rest_framework.permissions import AllowAny
from rest_framework.response import Response
from rest_framework.views import APIView

from ..serializers.login_serializer import LoginSerializer


@extend_schema_view(
    post=extend_schema(
        summary="Авторизация пользователя",
        responses={
            status.HTTP_200_OK: LoginSerializer
        }
    ),
)
class LoginView(APIView):
    # This view should be accessible also for unauthenticated users.
    permission_classes = (AllowAny,)
    serializer_class = LoginSerializer

    def post(self, request, format=None):
        serializer = self.serializer_class(data=self.request.data, context={'request': self.request})
        serializer.is_valid(raise_exception=True)
        user = serializer.validated_data['user']
        login(request, user)
        return Response(None, status=status.HTTP_202_ACCEPTED)
