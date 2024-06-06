from django_filters import FilterSet
from django_filters.rest_framework import CharFilter


class ProblemFilter(FilterSet):
    title = CharFilter(field_name='title', lookup_expr='icontains')
