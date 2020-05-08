#pragma once

#include <algorithm>
#include <execution>

// 実行ポリシー版------------------------------------------------------------------------------------------------

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto All_Of(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::all_of(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Any_Of(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::any_of(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto None_Of(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::none_of(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn, class _ExPo>
auto For_Each(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::for_each(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Size, class _Fn, class _ExPo>
auto For_Each_N(_Container& _Cont, _Size size, _Fn _Func, _ExPo&& _Exec)
{
	return (std::for_each_n(_Exec, _Cont.begin(), size, _Func));
}

template <class _Container, class _Ty, class _ExPo>
_NODISCARD auto Find(_Container& _Cont, const _Ty& _Val, _ExPo&& _Exec)
{
	return (std::find(_Exec, _Cont.begin(), _Cont.end(), _Val));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Find_If(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::find_if(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Find_If_Not(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::find_if_not(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Find_End(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func, _ExPo&& _Exec)
{
	return (std::find_end(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Find_End(_Container1& _Cont1, _Container2& _Cont2, _ExPo&& _Exec)
{
	return (std::find_end(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Find_First_Of(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func, _ExPo&& _Exec)
{
	return (std::find_first_of(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Find_First_Of(_Container1& _Cont1, _Container2& _Cont2, _ExPo&& _Exec)
{
	return (std::find_first_of(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Adjacent_Find(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::adjacent_find(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Count(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::count(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Count_If(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::count_if(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Mismatch(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func, _ExPo&& _Exec)
{
	return (std::mismatch(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _ExPo>
_NODISCARD auto Mismatch(_Container1& _Cont1, _Container2& _Cont2, _ExPo&& _Exec)
{
	return (std::mismatch(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Equal(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func, _ExPo&& _Exec)
{
	return (std::equal(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn, class _ExPo>
_NODISCARD auto Equal(_Container1& _Cont1, _Container2& _Cont2, _ExPo&& _Exec)
{
	return (std::equal(_Exec, _Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container, class _Fn, class _ExPo>
void Sort(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	(std::sort(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
void Sort(_Container& _Cont, _ExPo&& _Exec)
{
	(std::sort(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
void Stable_Sort(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	(std::stable_sort(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
void Stable_Sort(_Container& _Cont, _ExPo&& _Exec)
{
	(std::stable_sort(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Is_Sorted(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::is_sorted(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
_NODISCARD auto Is_Sorted(_Container& _Cont, _ExPo&& _Exec)
{
	return (std::is_sorted(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Min_Element(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::min_element(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
_NODISCARD auto Min_Element(_Container& _Cont, _ExPo&& _Exec)
{
	return (std::min_element(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Max_Element(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::max_element(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
_NODISCARD auto Max_Element(_Container& _Cont, _ExPo&& _Exec)
{
	return (std::max_element(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto MinMax_Element(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::minmax_element(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
_NODISCARD auto MinMax_Element(_Container& _Cont, _ExPo&& _Exec)
{
	return (std::minmax_element(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Ty, class _ExPo>
void Fill(_Container& _Cont, const _Ty& _Val, _ExPo&& _Exec)
{
	(std::fill(_Exec, _Cont.begin(), _Cont.end(), _Val));
}

template <class _Container, class _Diff, class _Ty, class _ExPo>
auto Fill_N(_Container& _Cont, const _Diff _Count_raw, const _Ty& _Val, _ExPo&& _Exec)
{
	return (std::fill_n(_Exec, _Cont.begin(), _Count_raw, _Val));
}

template <class _Container, class _Fn, class _ExPo>
void Generate(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	(std::generate(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Diff, class _Fn, class _ExPo>
auto Generate_N(_Container& _Cont, const _Diff _Count_raw, _Fn _Func, _ExPo&& _Exec)
{
	return (std::generate_n(_Exec, _Cont.begin(), _Count_raw, _Func));
}

template <class _Container, class _Ty, class _ExPo>
_NODISCARD auto Remove(_Container& _Cont, const _Ty& _Val, _ExPo&& _Exec)
{
	return (std::remove(_Exec, _Cont.begin(), _Cont.end(), _Val));
}

template <class _Container, class _Diff, class _Fn, class _ExPo>
_NODISCARD auto Remove_If(_Container& _Cont, const _Diff _Count_raw, _Fn _Func, _ExPo&& _Exec)
{
	return (std::remove_if(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
_NODISCARD auto Unique(_Container& _Cont, _ExPo&& _Exec)
{
	return (std::unique(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
_NODISCARD auto Unique(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (std::unique(_Exec, _Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _ExPo>
void Reverse(_Container& _Cont, _ExPo&& _Exec)
{
	(std::reverse(_Exec, _Cont.begin(), _Cont.end()));
}

template <class _Container, class _ExPo>
auto Rotate(_Container& _Cont, const size_t _MiddlePos, _ExPo&& _Exec)
{
	return (std::rotate(_Exec, _Cont.begin(), _Cont.begin() + _MiddlePos, _Cont.end()));
}

template <class _Container, class _ExPo>
auto Accumulate(_Container& _Cont, _ExPo&& _Exec)
{
	using Type = typename _Container::value_type;

	return (std::reduce(_Exec, _Cont.begin(), _Cont.end(), Type()));
}

template <class _Container, class _Ty, class _Fn, class _ExPo>
auto Accumulate(_Container& _Cont, _Ty _Val, _Fn _Func, _ExPo&& _Exec)
{
	return (std::reduce(_Exec, _Cont.begin(), _Cont.end(), _Val, _Func));
}

template <class _Container, class _Uty, class _ExPo>
auto Erase_Remove(_Container& _Cont, const _Uty& _Val, _ExPo&& _Exec)
{
	return (_Cont.erase(std::remove(_Exec, _Cont.begin(), _Cont.end(), _Val), _Cont.end()));
}

template <class _Container, class _Fn, class _ExPo>
auto Erase_Remove_If(_Container& _Cont, _Fn _Func, _ExPo&& _Exec)
{
	return (_Cont.erase(std::remove_if(_Exec, _Cont.begin(), _Cont.end(), _Func), _Cont.end()));
}

// 通常版--------------------------------------------------------------------------------------------------------

template <class _Container, class _Fn>
_NODISCARD auto All_Of(_Container& _Cont, _Fn _Func)
{
	return (std::all_of(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn>
_NODISCARD auto Any_Of(_Container& _Cont, _Fn _Func)
{
	return (std::any_of(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn>
_NODISCARD auto None_Of(_Container& _Cont, _Fn _Func)
{
	return (std::none_of(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn>
auto For_Each(_Container& _Cont, _Fn _Func)
{
	return (std::for_each(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Size, class _Fn>
auto For_Each_N(_Container& _Cont, _Size size, _Fn _Func)
{
	return (std::for_each_n(_Cont.begin(), size, _Func));
}

template <class _Container, class _Ty>
_NODISCARD auto Find(_Container& _Cont, const _Ty& _Val)
{
	return (std::find(_Cont.begin(), _Cont.end(), _Val));
}

template <class _Container, class _Fn>
_NODISCARD auto Find_If(_Container& _Cont, _Fn _Func)
{
	return (std::find_if(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn>
_NODISCARD auto Find_If_Not(_Container& _Cont, _Fn _Func)
{
	return (std::find_if_not(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Find_End(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func)
{
	return (std::find_end(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Find_End(_Container1& _Cont1, _Container2& _Cont2)
{
	return (std::find_end(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Find_First_Of(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func)
{
	return (std::find_first_of(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Find_First_Of(_Container1& _Cont1, _Container2& _Cont2)
{
	return (std::find_first_of(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container, class _Fn>
_NODISCARD auto Adjacent_Find(_Container& _Cont, _Fn _Func)
{
	return (std::adjacent_find(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn>
_NODISCARD auto Count(_Container& _Cont, _Fn _Func)
{
	return (std::count(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Fn>
_NODISCARD auto Count_If(_Container& _Cont, _Fn _Func)
{
	return (std::count_if(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Mismatch(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func)
{
	return (std::mismatch(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2>
_NODISCARD auto Mismatch(_Container1& _Cont1, _Container2& _Cont2)
{
	return (std::mismatch(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Equal(_Container1& _Cont1, _Container2& _Cont2, _Fn _Func)
{
	return (std::equal(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end(), _Func));
}

template <class _Container1, class _Container2, class _Fn>
_NODISCARD auto Equal(_Container1& _Cont1, _Container2& _Cont2)
{
	return (std::equal(_Cont1.begin(), _Cont1.end(), _Cont2.begin(), _Cont2.end()));
}

template <class _Container, class _Fn>
void Sort(_Container& _Cont, _Fn _Func)
{
	(std::sort(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
void Sort(_Container& _Cont)
{
	(std::sort(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn>
void Stable_Sort(_Container& _Cont, _Fn _Func)
{
	(std::stable_sort(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
void Stable_Sort(_Container& _Cont)
{
	(std::stable_sort(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn>
_NODISCARD auto Is_Sorted(_Container& _Cont, _Fn _Func)
{
	return (std::is_sorted(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
_NODISCARD auto Is_Sorted(_Container& _Cont)
{
	return (std::is_sorted(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn>
_NODISCARD auto Min_Element(_Container& _Cont, _Fn _Func)
{
	return (std::min_element(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
_NODISCARD auto Min_Element(_Container& _Cont)
{
	return (std::min_element(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn>
_NODISCARD auto Max_Element(_Container& _Cont, _Fn _Func)
{
	return (std::max_element(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
_NODISCARD auto Max_Element(_Container& _Cont)
{
	return (std::max_element(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn>
_NODISCARD auto MinMax_Element(_Container& _Cont, _Fn _Func)
{
	return (std::minmax_element(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
_NODISCARD auto MinMax_Element(_Container& _Cont)
{
	return (std::minmax_element(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Ty>
void Fill(_Container& _Cont, const _Ty& _Val)
{
	(std::fill(_Cont.begin(), _Cont.end(), _Val));
}

template <class _Container, class _Diff, class _Ty>
auto Fill_N(_Container& _Cont, const _Diff _Count_raw, const _Ty& _Val)
{
	return (std::fill_n(_Cont.begin(), _Count_raw, _Val));
}

template <class _Container, class _Fn>
void Generate(_Container& _Cont, _Fn _Func)
{
	(std::generate(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container, class _Diff, class _Fn>
auto Generate_N(_Container& _Cont, const _Diff _Count_raw, _Fn _Func)
{
	return (std::generate_n(_Cont.begin(), _Count_raw, _Func));
}

template <class _Container, class _Ty>
_NODISCARD auto Remove(_Container& _Cont, const _Ty& _Val)
{
	return (std::remove(_Cont.begin(), _Cont.end(), _Val));
}

template <class _Container, class _Diff, class _Fn>
_NODISCARD auto Remove_If(_Container& _Cont, const _Diff _Count_raw, _Fn _Func)
{
	return (std::remove_if(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
_NODISCARD auto Unique(_Container& _Cont)
{
	return (std::unique(_Cont.begin(), _Cont.end()));
}

template <class _Container, class _Fn>
_NODISCARD auto Unique(_Container& _Cont, _Fn _Func)
{
	return (std::unique(_Cont.begin(), _Cont.end(), _Func));
}

template <class _Container>
void Reverse(_Container& _Cont)
{
	(std::reverse(_Cont.begin(), _Cont.end()));
}

template <class _Container>
auto Rotate(_Container& _Cont, const size_t _MiddlePos)
{
	return (std::rotate(_Cont.begin(), _Cont.begin() + _MiddlePos, _Cont.end()));
}

template <class _Container>
auto Accumulate(_Container& _Cont)
{
	using Type = typename _Container::value_type;

	return (std::reduce(_Cont.begin(), _Cont.end(), Type()));
}

template <class _Container, class _Ty, class _Fn>
auto Accumulate(_Container& _Cont, _Ty _Val, _Fn _Func)
{
	return (std::reduce(_Cont.begin(), _Cont.end(), _Val, _Func));
}

template <class _Container, class _Uty>
auto Erase_Remove(_Container& _Cont, const _Uty& _Val)
{
	return (_Cont.erase(std::remove(_Cont.begin(), _Cont.end(), _Val), _Cont.end()));
}

template <class _Container, class _Fn>
auto Erase_Remove_If(_Container& _Cont, _Fn _Func)
{
	return (_Cont.erase(std::remove_if(_Cont.begin(), _Cont.end(), _Func), _Cont.end()));
}
