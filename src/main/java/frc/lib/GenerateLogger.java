package frc.lib;

import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.lang.reflect.Type;
import java.lang.reflect.TypeVariable;

import javax.lang.model.element.Modifier;

import org.littletonrobotics.junction.Logger;

import com.palantir.javapoet.MethodSpec;
import com.palantir.javapoet.TypeVariableName;



public class GenerateLogger {
    public static void main(String[] args) {
        Class<Logger> logger = Logger.class;
        Method[] methods = logger.getDeclaredMethods();
        for(Method m : methods) {
            if("recordOutput".equals(m.getName())) {
                Type[] parameterTypes = m.getGenericParameterTypes();
                Parameter[] parameters = m.getParameters();
                TypeVariable<Method>[] typeParameters = m.getTypeParameters();
                
                MethodSpec.Builder methodBuilder = MethodSpec.methodBuilder("recordOutput")
                    .addModifiers(Modifier.PUBLIC, Modifier.STATIC);

                for(TypeVariable<Method> i : typeParameters) {
                    methodBuilder.addTypeVariable(TypeVariableName.get(i).withBounds(i.getBounds()));
                }

                for(int i = 0; i < parameters.length; i ++) {
                    methodBuilder.addParameter(parameterTypes[i], parameters[i].getName());
                }

                methodBuilder.beginControlFlow("if (shouldLog())");
                methodBuilder.addCode("Logger.recordOutput(");
                for(int i = 0; i < parameters.length; i ++) {
                    methodBuilder.addCode("$L", parameters[i].getName());
                    if (i != parameters.length - 1) {
                        methodBuilder.addCode(", ");
                    }
                }
                methodBuilder.addCode(");\n");
                methodBuilder.endControlFlow();
                
                MethodSpec methodspec = methodBuilder.build();
                System.out.println(methodspec.toString());
            }
        }
    }
}
