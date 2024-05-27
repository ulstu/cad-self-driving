from rest_framework import serializers

from ..serializers.problem_serializer import ProblemSerializer
from ..serializers.tournament_serializer import TournamentSerializer


class LeaderboardSerializer(serializers.Serializer):
    """Сериализатор для лидерборда"""
    first_name = serializers.CharField()
    last_name = serializers.CharField()
    username = serializers.CharField()
    total_points = serializers.IntegerField()


class LeaderboardProblemSerializer(serializers.Serializer):
    """Сериализатор для лидерборда по задаче"""
    problem = ProblemSerializer()
    items = LeaderboardSerializer(many=True)


class LeaderboardTournamentSerializer(serializers.Serializer):
    """Сериализатор для лидерборда по задаче"""
    tournament = TournamentSerializer()
    items = LeaderboardSerializer(many=True)
