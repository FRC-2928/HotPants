package frc.robot;

import java.lang.reflect.ParameterizedType;
import java.util.Arrays;
import java.util.Map;

import javax.management.InstanceAlreadyExistsException;

public class Utils {
    private static Map<Class<?>, Object> singletons;

    public static <T> void ensureSingleton(T instance) {
        Class<T> klass = Utils.<T>classOf();

        if(Utils.singletons.containsKey(klass)) Log.panic(new InstanceAlreadyExistsException("Only one instance of " + klass.getSimpleName() + " is allowed."));

        Utils.singletons.put(klass, instance);
    }

    /// Get a singleton instance for T
    @SuppressWarnings("unchecked")
    public static <T> T getSingleton() {
        return (T)Utils.singletons.get(Utils.<T>classOf());
    }

    /// Get a singleton instance for T, calling its constructor with the given parameters if it does not exist
    @SuppressWarnings("unchecked")
    public static <T> T getSingletonInit(Object... constructorParameters) {
        Class<T> klass = Utils.<T>classOf();

        T instance = (T)Utils.singletons.get(klass);
        
        if(instance == null) {
            try {
                instance = klass.getDeclaredConstructor(Arrays.stream(constructorParameters).map(value -> value.getClass()).toArray(Class<?>[]::new)).newInstance(constructorParameters);
                Utils.singletons.put(klass, instance);
                return instance;
            } catch(Exception __) {
                return null;
            }
        } else return instance;
    }

    /// Gets the class of a generic T
    public static <T> Class<T> classOf() {
        class ClassOf<U> {
            @SuppressWarnings("unchecked")
            public Class<U> get() {
                return (Class<U>)((ParameterizedType)this.getClass().getGenericSuperclass()).getActualTypeArguments()[0];
            }
        }

        return new ClassOf<T>().get();
    }
}
