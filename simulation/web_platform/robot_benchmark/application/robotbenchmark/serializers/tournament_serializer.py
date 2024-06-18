from rest_framework import serializers

from robotbenchmark.models import Tournament
from robotbenchmark.serializers.user_serializer import UserSerializer


class TournamentSerializer(serializers.ModelSerializer):
    """Сериализатор для модель соревнования"""
    users = UserSerializer(many=True, read_only=True)

    class Meta:
        model = Tournament
        fields = '__all__'
