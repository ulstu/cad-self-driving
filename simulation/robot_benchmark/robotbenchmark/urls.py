from django.urls import include
from drf_spectacular.views import SpectacularAPIView, SpectacularSwaggerView, SpectacularRedocView

from rest_framework import routers
from django.urls import path
from rest_framework_simplejwt import views as jwt_views

from .views.command_view import CommandQueueView
from .views.finish_problem_view import FinishProblemView
from .views.leaderboard_by_problem_view import LeaderboardByProblemView
from .views.leaderboard_by_tournament_view import LeaderboardByTournamentView
from .views.leaderboard_view import LeaderboardView
from .views.token_obtain_view import CustomTokenObtainPairView
from .views.tournament_view import TournamentViewSet
from .views.tournament_user_view import TournamentUserViewSet
from .views.problem_user_view import ProblemUserViewSet
from .views.problem_view import ProblemViewSet
from .views.user_view import UserViewSet

router = routers.DefaultRouter()
router.register("users", UserViewSet)
router.register("problem", ProblemViewSet)
router.register("users-problem", ProblemUserViewSet)
router.register("users-tournament", TournamentUserViewSet)
router.register("tournament", TournamentViewSet)

urlpatterns = [
    path("", include(router.urls)),
    path('leaderboard/', LeaderboardView.as_view()),
    path('leaderboard/tournament/<int:tournament_id>/', LeaderboardByTournamentView.as_view()),
    path('leaderboard/problem/<int:problem_id>/', LeaderboardByProblemView.as_view()),
    path('commands/', CommandQueueView.as_view()),
    path('finish/<int:problemuser_id>', FinishProblemView.as_view()),

    path('token/', CustomTokenObtainPairView.as_view(), name='token_obtain_pair'),
    path('token/refresh/', jwt_views.TokenRefreshView.as_view(), name='token_refresh'),

    path('schema/', SpectacularAPIView.as_view(), name='schema'),
    path('swagger/', SpectacularSwaggerView.as_view(url_name='schema'), name='swagger-ui'),
    path('redoc/', SpectacularRedocView.as_view(url_name='schema'), name='redoc'),
]
