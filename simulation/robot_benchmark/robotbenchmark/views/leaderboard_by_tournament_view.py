from django.contrib.auth import get_user_model
from django.db.models import Sum
from rest_framework.response import Response
from rest_framework.views import APIView

from ..serializers.tournament_serializer import TournamentSerializer
from ..models import Tournament
from ..serializers.leaderboard_serializer import LeaderboardSerializer, LeaderboardTournamentSerializer


class LeaderboardByTournamentView(APIView):
    """Вью для лидерборда определённого турнира"""

    def get(self, request, tournament_id: int, format=None):
        """Return Leaderboard определённого турнира"""
        tournament = Tournament.objects.get(pk=tournament_id)
        user_model = get_user_model()
        users = user_model.objects.filter(user_tournaments__pk=tournament_id).annotate(total_points=Sum('tournamentuser__points')).order_by('-total_points')
        serializer = LeaderboardTournamentSerializer(data={
            'tournament': TournamentSerializer(tournament).data,
            'items': LeaderboardSerializer(users, many=True).data
        })
        serializer.is_valid(raise_exception=True)
        return Response(serializer.data)



