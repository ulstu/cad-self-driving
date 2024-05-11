from django_filters.rest_framework import DjangoFilterBackend
from drf_spectacular.utils import extend_schema, extend_schema_view
from rest_framework import viewsets, status
from rest_framework.filters import OrderingFilter
from rest_framework.permissions import IsAuthenticated

from ..filters.problem_filter import ProblemFilter
from ..models import Problem
from ..serializers.problem_serializer import ProblemSerializer


@extend_schema(tags=["problems"])
@extend_schema_view(
    retrieve=extend_schema(
        summary="Детальная информация о задаче",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    list=extend_schema(
        summary="Получение списка задач",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    update=extend_schema(
        summary="Обновление данных о задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    create=extend_schema(
        summary="Создание задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    destroy=extend_schema(
        summary="Удаление задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    ),
    partial_update=extend_schema(
        summary="Обновление с необязательными полями задачи",
        responses={
            status.HTTP_200_OK: ProblemSerializer
        }
    )
)
class ProblemViewSet(viewsets.ModelViewSet):
    queryset = Problem.objects.all()
    serializer_class = ProblemSerializer
    filter_backends = [DjangoFilterBackend, OrderingFilter]
    permission_classes = [IsAuthenticated]
    filterset_class = ProblemFilter

    # def retrieve(self, request, *args, **kwargs):
    #     instance = self.get_object()
    #     serializer = self.get_serializer(instance)
    #     init_url = 'ws://localhost:1999' + instance.world_path
    #     return Response(
    #         {'problem': serializer.data, "init_url": init_url}
    #     )

    # def home(request):
    #     problems = Problem.objects.order_by('difficulty').all()
    #     context = {'problems': problems}
    #     return render(request, 'robotbenchmark/problemList.html', context)
    #
    #
    # def problem(request, pk):
    #     problem = Problem.objects.get(id=pk)
    #     initUrl = 'ws://localhost:1999/' + problem.world_path
    #     context = {'problem': problem, 'initUrl': initUrl}
    #     return render(request, 'robotbenchmark/problem.html', context)
