from django.contrib.auth import get_user_model
from django.db.models import Sum
from rest_framework.response import Response
from rest_framework.views import APIView

from ..serializers.leaderboard_serializer import LeaderboardSerializer


class LeaderboardView(APIView):
    """Вью для лидерборда"""

    def get(self, request, format=None):
        """Return Общий Leaderboard"""
        user_model = get_user_model()
        users = user_model.objects.all().annotate(total_points=Sum('tournamentuser__points')).order_by('-total_points')
        serializer = LeaderboardSerializer(users, many=True)
        return Response(serializer.data)


