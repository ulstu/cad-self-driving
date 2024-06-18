from rest_framework import serializers

from robotbenchmark.models import Problem
from robotbenchmark.serializers.user_serializer import UserSerializer


class ProblemSerializer(serializers.ModelSerializer):
    """Сериализатор для модель соревнования"""
    users = UserSerializer(many=True, read_only=True)

    class Meta:
        model = Problem
        fields = '__all__'


class ProblemWithImageURLSerializer(serializers.ModelSerializer):
    """Сериализатор для модель соревнования"""
    users = UserSerializer(many=True, read_only=True)
    image = serializers.CharField()

    class Meta:
        model = Problem
        fields = '__all__'
