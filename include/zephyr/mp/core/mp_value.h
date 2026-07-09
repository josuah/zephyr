/*
 * Copyright 2025-2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Main header for mp_value.
 */

#ifndef ZEPHYR_INCLUDE_MP_CORE_MP_VALUE_H_
#define ZEPHYR_INCLUDE_MP_CORE_MP_VALUE_H_

/**
 * @defgroup mp_value Value Container
 * @ingroup mp_core
 * @brief A generic container for values for different @ref mp_value_type
 *
 * @{
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/mp/core/mp_object.h>

/** @brief Value comparison result: first value is less than second */
#define MP_VALUE_LESS_THAN      -1
/** @brief Value comparison result: values are equal */
#define MP_VALUE_EQUAL          0
/** @brief Value comparison result: first value is greater than second */
#define MP_VALUE_GREATER_THAN   1
/** @brief Value comparison result: values cannot be ordered */
#define MP_VALUE_UNORDERED      2
/** @brief Value comparison failed due to error */
#define MP_VALUE_COMPARE_FAILED 3

/**
 * @brief value encoded as either an immediate value, or a pointer to a value structure.
 */
typedef struct mp_value *mp_value_t;

/**
 * @brief mp_value type enumeration
 */
enum mp_value_type {
	MP_TYPE_NONE = 0,            /**< No type */
	MP_TYPE_BOOLEAN,             /**< Boolean value */
	MP_TYPE_ENUM,                /**< Enumeration value */
	MP_TYPE_INT,                 /**< Signed integer value */
	MP_TYPE_RANGE,               /**< Integer range value */
	MP_TYPE_STRING,              /**< String value */
	MP_TYPE_LIST,                /**< List of values */
	MP_TYPE_OBJECT,              /**< Object reference */
	MP_TYPE_PTR,                 /**< Pointer type */
	MP_TYPE_COUNT                /**< Number of types */
};

/** @brief Helper for passing a boolean to va_arg functions */
#define MP_BOOLEAN(value)	MP_TYPE_BOOLEAN, (int64_t)(value)

/** @brief Helper for passing an enum to va_arg functions */
#define MP_ENUM(value)		MP_TYPE_ENUM, (int64_t)(value)

/** @brief Helper for passing an integer to va_arg functions */
#define MP_INT(value)		MP_TYPE_INT, (int64_t)(value)

/** @brief Helper for passing a string to va_arg functions */
#define MP_STRING(value)	MP_TYPE_STRING, (char *)(value)

/** @brief Helper for passing a range to va_arg functions */
#define MP_RANGE(min, max, step) MP_TYPE_RANGE, (int64_t)(min), (int64_t)(max), (int64_t)(step)

/** @brief Helper for passing a list to va_arg functions */
#define MP_LIST(...)		MP_TYPE_LIST, __VA_ARGS__, NULL

/** @brief Helper for passing an mp_object to va_arg functions */
#define MP_OBJECT(value)	MP_TYPE_OBJECT, (struct mp_object *)(value)

/** @brief Helper for passing a pointer to va_arg functions */
#define MP_PTR(value)		MP_TYPE_PTR, (void *)(value)

/**
 * @brief Base mp_value structure
 */
struct mp_value {
	/** For internal use, see @ref mp_value_get_type */
	enum mp_value_type _type;
};

/**
 * @brief Create a new mp_value with the specified type and initialization arguments.
 *
 * This function creates a new mp_value instance based on the provided type.
 * The number and type of variadic arguments depend on the specified enum mp_value_type:
 *
 * - MP_TYPE_BOOLEAN, MP_TYPE_ENUM, MP_TYPE_INT, MP_TYPE_STRING, MP_TYPE_OBJECT, MP_TYPE_PTR:
 *   Require one initialization value.
 * - MP_TYPE_RANGE: Requires three integer values (min, max, and step).
 * - MP_TYPE_LIST: Requires a sequence of mp_value elements, terminated with NULL
 *   to indicate the end of the list.
 *
 * @param type The type of the value to create.
 * @param ... Variadic arguments used to initialize the value, depending on the specified type.
 *
 * @return Pointer to the newly created mp_value, NULL if memory allocation fails or an invalid
 *         type or argument list is provided.
 */
mp_value_t mp_value_new(enum mp_value_type type, ...);

/**
 * @brief Create a new mp_value from a va_list.
 *
 * Same as @ref mp_value_new but accepts a va_list pointer instead of variadic arguments.
 *
 * @param type The type of the value to create.
 * @param args Pointer to a va_list containing the initialization arguments.
 *
 * @return Pointer to the newly created mp_value, or NULL on failure.
 */
mp_value_t mp_value_new_va_list(enum mp_value_type type, va_list *args);

/**
 * @brief Create an empty value with given type.
 *
 * @param type type of value
 *
 * @return Pointer to the newly created mp_value, or NULL on failure.
 */
mp_value_t mp_value_new_empty(enum mp_value_type type);

/**
 * @brief Destroy a value and release its resources.
 *
 * @param value value to destroy
 *
 * @return 0 on success, -EINVAL if value is NULL
 */
int mp_value_destroy(mp_value_t value);

/**
 * @brief Get list size.
 *
 * @param list list of values
 *
 * @return size of list
 */
size_t mp_value_list_get_size(const mp_value_t list);

/**
 * @brief Return true if list is empty.
 *
 * @param list list of values
 *
 * @return true if list is empty, false otherwise
 */
bool mp_value_list_is_empty(const mp_value_t list);

/**
 * @brief Append value to list.
 *
 * @param list list to append to
 * @param append_value value to append
 *
 * @return 0 on success, -EINVAL if arguments are invalid, -ENOMEM on allocation failure
 */
int mp_value_list_append(mp_value_t list, mp_value_t append_value);

/**
 * @brief Set values to type.
 *
 * @param value pointer to the value to set
 * @param type type of value
 * @param ... Variadic arguments used to initialize the value,
 *            same rule as @ref mp_value_new()
 *
 * @return 0 on success, -EINVAL if value is NULL or type/arguments are invalid
 */
int mp_value_set(mp_value_t *value, int type, ...);

/**
 * @brief Get the type of a value
 *
 * @param value the value to query
 *
 * @return type of value
 */
enum mp_value_type mp_value_get_type(const mp_value_t value);

/**
 * @brief Set the type of a value
 *
 * @param value pointer the value
 * @param value type to set
 */
void mp_value_set_type(mp_value_t *value, enum mp_value_type type);

/**
 * @brief Get value at index in list.
 *
 * @param list list of value
 * @param index index of value to get from list
 *
 * @return value at given index in list, or NULL if not found
 */
mp_value_t mp_value_list_get(const mp_value_t list, int index);

/** Get boolean value of MP_TYPE_BOOLEAN */
bool mp_value_get_boolean(const mp_value_t value);

/** Get int value of MP_TYPE_INT */
int64_t mp_value_get_int(const mp_value_t value);

/** Get string value of MP_TYPE_STRING */
const char *mp_value_get_string(const mp_value_t value);

/** Get pointer value of MP_TYPE_PTR */
void *mp_value_get_ptr(const mp_value_t value);


/** Get minimum value of @ref mp_value with MP_TYPE_RANGE */
int64_t mp_value_get_range_min(const mp_value_t range);

/* Get maximum value of @ref mp_value with MP_TYPE_RANGE */
int64_t mp_value_get_range_max(const mp_value_t range);

/* Get step value of @ref mp_value with MP_TYPE_RANGE */
int64_t mp_value_get_range_step(const mp_value_t range);

/** Get the object reference of a mp_value with MP_TYPE_OBJECT */
struct mp_object *mp_value_get_object(mp_value_t value);

/**
 * Comparison between two primitive values
 *
 * @param val1 first value
 * @param val2 second value
 * @return MP_VALUE_GREATER_THAN if val1 > val2
 *	MP_VALUE_LESS_THAN if val1 < val2
 *	MP_VALUE_EQUAL if val1 == val2
 *	MP_VALUE_UNORDERED if val1 and val2 are not comparable
 *	MP_VALUE_COMPARE_FAILED if val1 and val2 are not same type
 */
int mp_value_compare(const mp_value_t val1, const mp_value_t val2);

/**
 * Intersect between two values
 *
 * @param val1 reference value to compare with
 * @param val2 value to compare with
 * @return NULL if intersect is empty
 */
mp_value_t mp_value_intersect(const mp_value_t val1, const mp_value_t val2);

/**
 * Intersect between value and range
 *
 * @param ref_val reference value to compare with
 * @param compare_val value to compare with
 * @return NULL if intersect is empty
 */
mp_value_t mp_value_intersect_range(const mp_value_t ref_val,
				    const mp_value_t compare_val);

/**
 * Intersect between list with value, range or list
 *
 * @param list reference value to compare with
 * @param compare_val value to compare with
 * @return NULL if intersect is empty
 */
mp_value_t mp_value_intersect_list(const mp_value_t list,
					 const mp_value_t compare_val);

/**
 * Check if two values can intersect
 *
 * @param val1 first value
 * @param val2 second value
 * @return true if two values can intersect
 */
bool mp_value_can_intersect(const mp_value_t val1, const mp_value_t val2);

/**
 * Duplicate value
 *
 * @param value value to duplicate
 * @return new value with same type and data as original value, or NULL on failure
 * @note For string only pointer is copied, not string itself.
 */
mp_value_t mp_value_duplicate(const mp_value_t value);

/**
 * @brief Check if a value is a primitive type
 *
 * @param value Value to check, must not be NULL
 *
 * @return true if value is primitive, false otherwise
 */
bool mp_value_is_primitive(const mp_value_t value);

/**
 * @brief Print a value
 *
 * @param value Value to print, may be NULL
 * @param new_line Add newline after printing
 */
void mp_value_print(const mp_value_t value, bool new_line);

/** @} */

#endif /*ZEPHYR_INCLUDE_MP_CORE_MP_VALUE_H_*/
