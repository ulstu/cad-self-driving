from rest_framework import serializers

from ..serializers.problem_serializer import ProblemSerializer, ProblemWithImageURLSerializer
from ..serializers.tournament_serializer import TournamentSerializer


class LeaderboardSerializer(serializers.Serializer):
    """Сериализатор для лидерборда"""
    first_name = serializers.CharField(required=False, allow_blank=True)
    last_name = serializers.CharField(required=False, allow_blank=True)
    username = serializers.CharField()
    total_points = serializers.IntegerField()


class LeaderboardProblemSerializer(serializers.Serializer):
    """Сериализатор для лидерборда по задаче"""
    problem = ProblemWithImageURLSerializer()
    items = LeaderboardSerializer(many=True)


class LeaderboardTournamentSerializer(serializers.Serializer):
    """Сериализатор для лидерборда по задаче"""
    tournament = TournamentSerializer()
    items = LeaderboardSerializer(many=True)
