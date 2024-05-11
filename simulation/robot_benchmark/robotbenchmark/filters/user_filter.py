from django_filters import FilterSet
from django_filters.rest_framework import CharFilter


class UserFilter(FilterSet):
    username = CharFilter(field_name='username', lookup_expr='icontains')
