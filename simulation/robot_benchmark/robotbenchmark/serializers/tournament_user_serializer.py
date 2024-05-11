from rest_framework import serializers

from robotbenchmark.models import TournamentUser


class TournamentUserSerializer(serializers.ModelSerializer):
    """Сериализатор для модель many-to-many Соревнования-Пользователи"""

    class Meta:
        model = TournamentUser
        fields = '__all__'
