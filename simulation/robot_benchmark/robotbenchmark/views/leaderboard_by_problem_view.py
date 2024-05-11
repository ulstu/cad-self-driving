from django.contrib.auth import get_user_model
from django.db.models import Sum, F
from rest_framework.response import Response
from rest_framework.views import APIView

from ..serializers.problem_serializer import ProblemSerializer
from ..models import Problem
from ..serializers.leaderboard_serializer import LeaderboardProblemSerializer, LeaderboardSerializer


class LeaderboardByProblemView(APIView):
    """Вью для лидерборда определённой задачи"""

    def get(self, request, problem_id: int, format=None):
        """Return Leaderboard определённой задачи"""
        problem = Problem.objects.get(pk=problem_id)
        user_model = get_user_model()
        users = user_model.objects.filter(user_problems__pk=problem_id).annotate(total_points=F('problemuser__points')).order_by('-total_points')
        serializer = LeaderboardProblemSerializer(data={
            'problem': ProblemSerializer(problem).data,
            'items': LeaderboardSerializer(users, many=True).data
        })
        serializer.is_valid(raise_exception=True)
        return Response(serializer.data)



